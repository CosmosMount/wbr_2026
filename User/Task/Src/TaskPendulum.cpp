#include "bsp_dwt.hpp"
#include "math.hpp"
#include "pid.hpp"
#include "lqr.hpp"
#include "slope.hpp"
#include "magicmsgs.hpp"
#include "config_chassis.hpp"
#include "tx_api.h"
#include "vmc.hpp"
#include "om.h"
#include <cstdlib>

TX_THREAD PendulumThread;
TX_SEMAPHORE PendulumThreadSem;
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
    float llen;
    float rlen;
    float l_ref;
    float pitch;
    float pitch_dot;
    float Twl;
    float Twr;
    float Tpl;
    float Tpr;
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
    bool lneutral;
    bool rneutral;
};

struct pid_tuning_t {
    float kp;
    float ki;
    float kd;
};

msg_ins_t debug_ins; 
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
    PID rleg_len_pd(2000.0f, 0.0f, -1000.0f, 200.0f, 0.0f, PID_DVEL);
    PID lleg_len_pd(2000.0f, 0.0f, -1000.0f, 200.0f, 0.0f, PID_DVEL);

    /* Roll Params Initialization */
    PID roll_pd(0.7f, 0.0f, 0.01f, 3.0f, 0.0f);
    SLOPE roll_updater(0.0f,0.0002f);

    /* LQR Initialization */
    LQR lqr_controller;
    float Tout[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float observedX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float refX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    lqr_controller.InitMatX(&refX[0], &observedX[0]);
    float llenref = 0.12f;
    float rlenref = 0.12f;

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
    lenpd_tuning.kp = 2000.0f;
    lenpd_tuning.ki = 0.0f;
    lenpd_tuning.kd = -1000.0f;
    phi0pd_tuning.kp = 20.0f;
    phi0pd_tuning.ki = 0.0f;
    phi0pd_tuning.kd = 10.0f;
    yawpd_tuning.kp = 10.0f;
    yawpd_tuning.ki = 0.0f;
    yawpd_tuning.kd = 2.0f;
#endif

    for (;;)
    {
        float thread_start_time = tx_time_get();
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(solver_suber, &solver_fdb, false);
        om_suber_export(odom_suber, &odom, false);
        om_suber_export(cmd_suber, &cmd, false);

        if (tx_semaphore_get(&IMUThreadSem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            observedX[0] = odom.x;
            observedX[1] = odom.v;
            observedX[2] = ins.total_yaw*DegreeToRad;
            observedX[3] = ins.gyro_y;
            observedX[4] = solver_fdb.lalpha;
            observedX[5] = solver_fdb.lalpha_dot;
            observedX[6] = solver_fdb.ralpha;
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

                if (solver_fdb.lneutral && solver_fdb.rneutral)
                {
                    refX[0] = cmd.x;
                    refX[1] = cmd.v;
                    llenref = cmd.len+0.03f+roll_pd.result;
                    rlenref = cmd.len+0.03f-roll_pd.result;
                }
                else 
                {
                    refX[0] = odom.x;
                    refX[1] = odom.v;

                    if (!solver_fdb.lneutral)
                    {
                        llenref = 0.12f;
                    }
                    else 
                    {
                        llenref = solver_fdb.llen;
                    }

                    if (!solver_fdb.rneutral)
                    {
                        rlenref = 0.12f;
                    }
                    else 
                    {
                        rlenref = solver_fdb.rlen;
                    }
                }

                lleg_len_pd.ref = llenref;
                lleg_len_pd.fdb = solver_fdb.llen;
                lleg_len_pd.UpdateResult(solver_fdb.llen_dot);
                pendulum_ctrl.Tl[0] = lleg_len_pd.result;

                rleg_len_pd.ref = rlenref;
                rleg_len_pd.fdb = solver_fdb.rlen;
                rleg_len_pd.UpdateResult(solver_fdb.rlen_dot);
                pendulum_ctrl.Tr[0] = rleg_len_pd.result;
                
                
                refX[2] = ins.total_yaw*DegreeToRad+cmd.dyaw*0.001f;
                refX[3] = cmd.w+cmd.dyaw;
                refX[4] = 0.14f;
                refX[5] = 0.0f;
                refX[6] = 0.14f;
                refX[7] = 0.0f;
                refX[8] = 0.0f;
                refX[9] = 0.0f;

                lqr_controller.refreshLQRK(solver_fdb.llen, solver_fdb.rlen);
                lqr_controller.LQRCal(Tout);

                pendulum_ctrl.Twl = Tout[0];
                pendulum_ctrl.Twr = Tout[1];
                pendulum_ctrl.Tl[1] = Tout[2];
                pendulum_ctrl.Tr[1] = Tout[3];

            }
        }

    #ifdef DEBUG
        debug_ins = ins;
        pendulum_debug.alphal = observedX[4];
        pendulum_debug.alphal_dot = observedX[5];
        pendulum_debug.alphar = observedX[6];
        pendulum_debug.alphar_dot = observedX[7];
        pendulum_debug.pitch = ins.pitch*DegreeToRad;
        pendulum_debug.pitch_dot = ins.gyro_p;
        pendulum_debug.Twl = Tout[0];
        pendulum_debug.Twr = Tout[1];
        pendulum_debug.Tpl = Tout[2];
        pendulum_debug.Tpr = Tout[3];
        pendulum_debug.Fl = pendulum_ctrl.Tl[0];
        pendulum_debug.Fr = pendulum_ctrl.Tr[0];
        pendulum_debug.llen = solver_fdb.llen;
        pendulum_debug.rlen = solver_fdb.rlen;
        pendulum_debug.l_ref = cmd.len;
        pendulum_debug.x = odom.x;
        pendulum_debug.xref = refX[0];
        pendulum_debug.v = odom.v;
        pendulum_debug.vref = refX[1];
        pendulum_debug.delta_phi = solver_fdb.lphi - solver_fdb.rphi;
        pendulum_debug.llendot = solver_fdb.llen_dot;
        pendulum_debug.yaw = ins.total_yaw*DegreeToRad;
        pendulum_debug.yawref = refX[2];
        pendulum_debug.yaw_dot = ins.gyro_r;
        pendulum_debug.lneutral = solver_fdb.lneutral;
        pendulum_debug.rneutral = solver_fdb.rneutral;
        lleg_len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
        rleg_len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
    #endif
        
        om_publish(pendulumctrl_topic, &pendulum_ctrl, sizeof(msg_ctrl_t), true, false);
        tx_semaphore_put(&PendulumThreadSem);
        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));
    }
}
