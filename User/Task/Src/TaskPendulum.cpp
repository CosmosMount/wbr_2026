#include "bsp_dwt.hpp"
#include "math.hpp"
#include "pid.hpp"
#include "lqr.hpp"
#include "slope.hpp"
#include "magicmsgs.hpp"
#include "config_chassis.hpp"
#include "vmc.hpp"
#include "om.h"

#ifdef SJTU_MODEL

TX_THREAD PendulumThread;
uint8_t PendulumThreadStack[4096] = {0};

extern TX_SEMAPHORE IMUThreadSem;

#ifdef DEBUG
struct pendulum_debug_t
{
    float alphal;
    float alphal_dot;
    float alphar;
    float alphar_dot;
    float x;
    float xref;
    float v;
    float vref;
    float l;
    float l_ref;
    float pitch;
    float pitch_dot;
    float T;
    float Tp;
    float Fl;
    float Fr;
    float delta_phi;
    float Cphi;
    float llendot;
    float rlendot;
    float yaw;
    float yawref;
    float yaw_dot;
    float yaw_out;
};

struct pid_tuning_t {
    float kp;
    float ki;
    float kd;
};

msg_ins_t debug_ins; //__attribute__((section(".RAM_D3"))) 
pendulum_debug_t pendulum_debug;
pid_tuning_t lenpd_tuning;
pid_tuning_t phi0pd_tuning;
pid_tuning_t yawpd_tuning;
float last_alpha = 0.0f;
float debug_alpha_dot = 0.0f;
#endif

[[noreturn]] void PendulumThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    /* Legs Params Initialization */
    PID rleg_len_pd(5000.0f, 0.0f, -8000.0f, 200.0f, 0.0f, PID_DVEL);
    PID lleg_len_pd(5000.0f, 0.0f, -8000.0f, 200.0f, 0.0f, PID_DVEL);

    /* Roll Params Initialization */
    PID roll_pd(0.7f, 0.0f, 0.01f, 3.0f, 0.0f);
    SLOPE roll_updater(0.0f,0.0002f);

    /* LQR Initialization */
    LQR lqr_controller;
    float Tout[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float observedX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float refX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    lqr_controller.InitMatX(&refX[0], &observedX[0]);

    float lenref = 0.21f;
    float thread_start_time;

    /* One Message Initialization */
    om_topic_t *pendulumctrl_topic =om_config_topic(nullptr, "ca", "pendulumctrl", sizeof(msg_ctrl_t));
    msg_ctrl_t pendulum_ctrl{};

    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *solver_suber = om_subscribe(om_find_topic("solverfdb", UINT32_MAX));
    msg_solver_t solver_fdb{};
    om_suber_t *odom_suber = om_subscribe(om_find_topic("odom", UINT32_MAX));
    msg_odometry_t odom{};
    om_suber_t *cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
    msg_cmd_t cmd{};

#ifdef DEBUG
    lenpd_tuning.kp = 4000.0f;
    lenpd_tuning.ki = 0.0f;
    lenpd_tuning.kd = -120.0f;
    phi0pd_tuning.kp = 20.0f;
    phi0pd_tuning.ki = 0.0f;
    phi0pd_tuning.kd = 10.0f;
    yawpd_tuning.kp = 10.0f;
    yawpd_tuning.ki = 0.0f;
    yawpd_tuning.kd = 2.0f;
#endif

    for (;;)
    {
        thread_start_time = tx_time_get();
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(solver_suber, &solver_fdb, false);
        om_suber_export(odom_suber, &odom, false);
        om_suber_export(cmd_suber, &cmd, false);

        if (tx_semaphore_get(&IMUThreadSem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            observedX[0] = odom.x;//0.0f;//
            observedX[1] = odom.v;//0.0f;//
            observedX[2] = ins.total_yaw*DegreeToRad;//0.0f;//
            observedX[3] = ins.gyro_y;//0.0f;//
            observedX[4] = solver_fdb.lalpha;//0.0f;//
            observedX[5] = solver_fdb.lalpha_dot;
            observedX[6] = solver_fdb.ralpha;//0.0f;//
            observedX[7] = solver_fdb.ralpha_dot;
            observedX[8] = ins.pitch*DegreeToRad;
            observedX[9] = ins.gyro_p;
            
            if (!cmd.move)
            {
                pendulum_ctrl.Tl[0] = 0.0f;
                pendulum_ctrl.Tr[0] = 0.0f;
                pendulum_ctrl.Tl[1] = 0.0f;
                pendulum_ctrl.Tr[1] = 0.0f;
                pendulum_ctrl.Twl = 0.0f;
                pendulum_ctrl.Twr = 0.0f;
                Tout[0] = 0.0f;
                Tout[1] = 0.0f;
                Tout[2] = 0.0f;
                Tout[3] = 0.0f;
                refX[0] = odom.x;
                refX[1] = odom.v;
                refX[2] = ins.total_yaw*DegreeToRad;
                refX[3] = 0.0f;
                refX[4] = 0.0f;
                refX[5] = 0.0f;
                refX[6] = 0.0f;
                refX[7] = 0.0f;
                refX[8] = 0.0f;
                refX[9] = 0.0f;
            }
            else 
            {
                roll_pd.ref = cmd.roll;
                roll_pd.fdb = ins.roll*DegreeToRad;
                roll_pd.UpdateResult(ins.gyro_r);

                lleg_len_pd.ref = lenref+0.03f+roll_pd.result;
                lleg_len_pd.fdb = solver_fdb.llen;
                lleg_len_pd.UpdateResult(solver_fdb.llen_dot);
                pendulum_ctrl.Tl[0] = lleg_len_pd.result;//0.0f;//

                rleg_len_pd.ref = lenref+0.03f-roll_pd.result;
                rleg_len_pd.fdb = solver_fdb.rlen;
                rleg_len_pd.UpdateResult(solver_fdb.rlen_dot);
                pendulum_ctrl.Tr[0] = rleg_len_pd.result;//0.0f;//
                
                refX[0] = cmd.x;
                refX[1] = cmd.v;
                refX[2] = ins.total_yaw*DegreeToRad+cmd.dyaw*0.001f;
                refX[3] = cmd.w+cmd.dyaw;
                refX[4] = 0.0f;
                refX[5] = 0.0f;
                refX[6] = 0.0f;
                refX[7] = 0.0f;
                refX[8] = 0.0f;
                refX[9] = 0.0f;

                lqr_controller.refreshLQRK(solver_fdb.llen, solver_fdb.rlen, solver_fdb.N<20.0f);
                lqr_controller.LQRCal(Tout);
                pendulum_ctrl.Twl = Tout[0];
                pendulum_ctrl.Twr = Tout[1];
                pendulum_ctrl.Tl[1] = Tout[2];//0.0f;//
                pendulum_ctrl.Tr[1] = Tout[3];//0.0f;//
            }
        }

    #ifdef DEBUG
        debug_ins = ins;
        // debug_remoter = remoter;
        pendulum_debug.alphal = observedX[4];
        pendulum_debug.alphal_dot = observedX[5];
        pendulum_debug.pitch = ins.pitch*DegreeToRad;
        pendulum_debug.pitch_dot = ins.gyro_p;
        pendulum_debug.T = Tout[0];
        pendulum_debug.Tp = Tout[1];
        pendulum_debug.Fl = pendulum_ctrl.Tl[0];
        pendulum_debug.Fr = pendulum_ctrl.Tr[0];
        // pendulum_debug.l = lenfdb;
        pendulum_debug.l_ref = lenref;
        pendulum_debug.x = odom.x;
        pendulum_debug.xref = refX[0];
        pendulum_debug.v = odom.v;
        pendulum_debug.vref = refX[1];
        pendulum_debug.delta_phi = solver_fdb.lphi - solver_fdb.rphi;
        pendulum_debug.llendot = solver_fdb.llen_dot;
        pendulum_debug.yaw = ins.total_yaw*DegreeToRad;
        pendulum_debug.yawref = refX[2];
        pendulum_debug.yaw_dot = ins.gyro_r;
        lleg_len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
        rleg_len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
    #endif
        
        om_publish(pendulumctrl_topic, &pendulum_ctrl, sizeof(msg_ctrl_t), true, false);
        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));
    }
}
#else
TX_THREAD PendulumThread;
uint8_t PendulumThreadStack[4096] = {0};

uint8_t xyAndRefAngleMsg[8] = {0};
uint8_t chassisStateMsg[8] = {0};

extern TX_SEMAPHORE IMUThreadSem;

#ifdef DEBUG
struct pendulum_debug_t
{
    float alpha;
    float alpha_dot;
    float x;
    float xref;
    float v;
    float vref;
    float l;
    float l_ref;
    float pitch;
    float pitch_dot;
    float T;
    float Tp;
    float Fl;
    float Fr;
    float delta_phi;
    float Cphi;
    float llendot;
    float rlendot;
    float yaw;
    float yawref;
    float yaw_dot;
    float yaw_out;
};

struct pid_tuning_t {
    float kp;
    float ki;
    float kd;
};

__attribute__((section(".RAM_D3"))) msg_ins_t debug_ins;
__attribute__((section(".RAM_D3"))) msg_remoter_t debug_remoter;
pendulum_debug_t pendulum_debug;
pid_tuning_t lenpd_tuning;
pid_tuning_t phi0pd_tuning;
pid_tuning_t yawpd_tuning;
float last_alpha = 0.0f;
float debug_alpha_dot = 0.0f;
#endif

[[noreturn]] void PendulumThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    /* Legs Params Initialization */
    PID rleg_len_pd(5000.0f, 0.0f, -8000.0f, 200.0f, 0.0f, PID_DVEL);
    PID lleg_len_pd(5000.0f, 0.0f, -8000.0f, 200.0f, 0.0f, PID_DVEL);
    PID phi0_pd(50.0f, 0.0f, 10.0f, 50.0f, 0.0f);
    
    IIRFilter leg_len_filter(2,LOWPASS,1);
    SLOPE leg_len_updater(0.13f,0.0001f);

    /* Roll Params Initialization */
    PID roll_pd(0.7f, 0.0f, 0.01f, 3.0f, 0.0f);
    SLOPE roll_updater(0.0f,0.0002f);
    /* Yaw Params Initialization */
    PID yaw_pd(10.0f, 0.0f, 2.0f, 5.0f, 0.0f,PID_DVEL);
    PID spin_pd(0.8f, 0.0f, 0.6f, 4.5f, 0.0f);
    SLOPE yaw_updater(0.0f, 0.01f);
    /* Speed Params Initialization */
    SLOPE v_updater(0.0f,0.01f);
    /* LQR Initialization */
    LQR lqr_controller;
    float Tout[2] = {0.0f, 0.0f};
    float observedX[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float refX[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    lqr_controller.InitMatX(&refX[0], &observedX[0]);

    /* Control Signal Initialization */
    chassis_mode_t mode = {NONE, NORMAL_ROTATE, DO_NOT_JUMP, NOT_FLY_MODE};
    // uint16_t vlen_rx;
    // uint16_t v_rx;
    // float relativeangle;

    // float vlen;
    float v;
    float x_maintain;
    float lenfdb;
    float lenref = 0.21f;
    float alpha_fdb;
    float alphadot_fdb;
    float thread_start_time;
    bool stop_flag = false;
    bool maintained_x = false;

    /* One Message Initialization */
    om_topic_t *pendulumctrl_topic =om_config_topic(nullptr, "ca", "pendulumctrl", sizeof(msg_ctrl_t));
    msg_ctrl_t pendulum_ctrl{};

    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *solver_suber = om_subscribe(om_find_topic("solverfdb", UINT32_MAX));
    msg_solver_t solver_fdb{};
    om_suber_t *odom_suber = om_subscribe(om_find_topic("odom", UINT32_MAX));
    msg_odometry_t odom{};
    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};

#ifdef DEBUG
    lenpd_tuning.kp = 4000.0f;
    lenpd_tuning.ki = 0.0f;
    lenpd_tuning.kd = -120.0f;
    phi0pd_tuning.kp = 20.0f;
    phi0pd_tuning.ki = 0.0f;
    phi0pd_tuning.kd = 10.0f;
    yawpd_tuning.kp = 10.0f;
    yawpd_tuning.ki = 0.0f;
    yawpd_tuning.kd = 2.0f;
#endif

    for (;;)
    {
        thread_start_time = tx_time_get();
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(solver_suber, &solver_fdb, false);
        om_suber_export(odom_suber, &odom, false);
        om_suber_export(remoter_suber, &remoter, false);

        alpha_fdb = 0.5f*(solver_fdb.lphi + solver_fdb.rphi - Pi) + ins.pitch*DegreeToRad;
        alphadot_fdb = 0.5f*(solver_fdb.lphi_dot + solver_fdb.rphi_dot) + ins.gyro_p;
        lenfdb = 0.5f * (solver_fdb.llen + solver_fdb.rlen);

        if (tx_semaphore_get(&IMUThreadSem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            if (remoter.ctrl_sw == Relax || remoter.offline)
            {
                pendulum_ctrl.Tl[0] = 0.0f;
                pendulum_ctrl.Tr[0] = 0.0f;
                pendulum_ctrl.Tl[1] = 0.0f;
                pendulum_ctrl.Tr[1] = 0.0f;
                pendulum_ctrl.Twl = 0.0f;
                pendulum_ctrl.Twr = 0.0f;
                Tout[0] = 0.0f;
                Tout[1] = 0.0f;
                refX[2] = odom.x;
                refX[3] = 0.0f;
            }
            
            else 
            {
                // uint8_t state_msg = chassisStateMsg[0];
                // mode.chassis_mode = static_cast<chassis_mode_e>(state_msg & 0x03);
                // mode.rotate_type = static_cast<rotate_ctrl_e>((state_msg >> 2) & 0x01);
                // mode.jump_ctrl = static_cast<jump_ctrl_e>((state_msg >> 3) & 0x03);
                // mode.fly_ctrl = static_cast<fly_ctrl_e>((state_msg >> 5) & 0x01);

                // memcpy(&vlen_rx, xyAndRefAngleMsg, 2);                  // 将接收到的数据拷贝到Vx
                // memcpy(&v_rx, xyAndRefAngleMsg + 2, 2);                 // 将接收到的数据拷贝到Vy
                // memcpy(&relativeangle, xyAndRefAngleMsg + 4, 4);        // 将接收到的数据拷贝到RelativeAngle

                // // Vx Vy映射,[0,60000] -> [-2,2]
                // vlen = ((float)vlen_rx) / 15000.0f - 2.0f;
                // v = ((float)v_rx) / 15000.0f - 2.0f;

                // if (isnan(vlen_rx) || isnan(v_rx) || isnan(relativeangle) || (mode.chassis_mode > 3) || (mode.rotate_type > 1) || (mode.jump_ctrl > 2)) // 如果出现nan错误，将速度设定值设为0
                // {
                //     vlen = 0;
                //     v = 0;
                //     relativeangle = 0;
                //     mode.chassis_mode = NONE;
                //     mode.rotate_type = NORMAL_ROTATE;
                //     mode.jump_ctrl = DO_NOT_JUMP;
                // }

                // if (fabsf(vlen) < 0.0005f)
                //     vlen = 0;
                // if (fabsf(v) < 0.0005f)
                //     v = 0;
                // if (fabsf(relativeangle) < 0.0001f)
                //     relativeangle = 0;

                v = v_updater.UpdateVal(remoter.left_y *0.5f);
                stop_flag = (fabsf(v) < 0.002f);

                roll_pd.ref = remoter.left_x*0.1f;
                roll_pd.fdb = ins.roll*DegreeToRad;
                roll_pd.UpdateResult(ins.gyro_r);

                lleg_len_pd.ref = lenref+0.03f+roll_pd.result;
                lleg_len_pd.fdb = solver_fdb.llen;
                lleg_len_pd.UpdateResult(solver_fdb.llen_dot);
                pendulum_ctrl.Tl[0] = lleg_len_pd.result;//0.0f;//

                rleg_len_pd.ref = lenref+0.03f-roll_pd.result;
                rleg_len_pd.fdb = solver_fdb.rlen;
                rleg_len_pd.UpdateResult(solver_fdb.rlen_dot);
                pendulum_ctrl.Tr[0] = rleg_len_pd.result;//0.0f;//

                observedX[0] = alpha_fdb;//0.0f;//
                observedX[1] = alphadot_fdb;//0.0f;//
                observedX[2] = odom.x;//0.0f;//
                observedX[3] = odom.v;//0.0f;//
                observedX[4] = ins.pitch*DegreeToRad;
                observedX[5] = ins.gyro_p;

                refX[0] = 0.0f;
                refX[1] = 0.0f;
                if (stop_flag)
                {
                    if (!maintained_x)
                    {
                        x_maintain = odom.x;
                        maintained_x = true;
                    }
                    refX[2] = x_maintain;
                    refX[3] = 0.0f;
                }
                else
                {   
                    maintained_x = false;
                    refX[2] = odom.x+v*0.001f;
                    refX[3] = v;
                }
                refX[4] = 0.0f;
                refX[5] = 0.0f;

                lqr_controller.refreshLQRK(lenfdb, mode.fly_ctrl == FLY_MODE);
                lqr_controller.LQRCal(Tout);
                pendulum_ctrl.Tl[1] = Tout[1]*0.5f;//0.0f;//
                pendulum_ctrl.Tr[1] = Tout[1]*0.5f;//0.0f;//
                pendulum_ctrl.Twl = Tout[0]*0.5f;
                pendulum_ctrl.Twr = Tout[0]*0.5f;

                phi0_pd.ref = 0.0f;
                phi0_pd.fdb = solver_fdb.lphi - solver_fdb.rphi;
                phi0_pd.UpdateResult();
                pendulum_ctrl.Tl[1] += phi0_pd.result;//0.0f;//
                pendulum_ctrl.Tr[1] -= phi0_pd.result;//0.0f;//

                yaw_pd.ref = ins.total_yaw*DegreeToRad + remoter.right_x*0.1f;
                yaw_pd.fdb = ins.total_yaw*DegreeToRad;
                yaw_pd.UpdateResult(ins.gyro_y);
                pendulum_ctrl.Twl += yaw_pd.result;
                pendulum_ctrl.Twr -= yaw_pd.result;
            }
        }

    #ifdef DEBUG
        debug_ins = ins;
        debug_remoter = remoter;
        pendulum_debug.alpha = alpha_fdb;
        pendulum_debug.alpha_dot = alphadot_fdb;
        pendulum_debug.pitch = ins.pitch*DegreeToRad;
        pendulum_debug.pitch_dot = ins.gyro_p;
        pendulum_debug.T = Tout[0];
        pendulum_debug.Tp = Tout[1];
        pendulum_debug.Fl = pendulum_ctrl.Tl[0];
        pendulum_debug.Fr = pendulum_ctrl.Tr[0];
        pendulum_debug.l = lenfdb;
        pendulum_debug.l_ref = lenref;//leg_len_updater.UpdateVal(lenref);
        pendulum_debug.x = odom.x;
        pendulum_debug.xref = refX[2];
        pendulum_debug.v = odom.v;
        pendulum_debug.vref = refX[3];
        pendulum_debug.delta_phi = solver_fdb.lphi - solver_fdb.rphi;
        pendulum_debug.Cphi = phi0_pd.result;
        pendulum_debug.llendot = solver_fdb.llen_dot;
        pendulum_debug.yaw = ins.total_yaw*DegreeToRad;
        pendulum_debug.yawref = yaw_pd.ref;
        pendulum_debug.yaw_dot = ins.gyro_y;
        pendulum_debug.yaw_out = yaw_pd.result;
        lleg_len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
        rleg_len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
        phi0_pd.Tuning(phi0pd_tuning.kp, phi0pd_tuning.ki, phi0pd_tuning.kd);
        yaw_pd.Tuning(yawpd_tuning.kp, yawpd_tuning.ki, yawpd_tuning.kd);
    #endif
        
        om_publish(pendulumctrl_topic, &pendulum_ctrl, sizeof(msg_ctrl_t), true, false);
        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));
    }
}
#endif