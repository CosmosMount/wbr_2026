#include "DMMotor.hpp"
#include "arm_math_types.h"
#include "main.h"
#include "slope.hpp"
#include "tx_api.h"
#include "om.h"

#include "bsp_can.hpp"
#include "bsp_dwt.hpp"

#include "math.hpp"
#include "M3508.hpp"
#include "DM8009P.hpp"
#include "DMMotorHandler.hpp"
#include "DJIMotorHandler.hpp"

#include "lqr.hpp"
#include "odometry.hpp"
#include "pendulum.hpp"
#include "magicmsgs.hpp"
#include "config_chassis.hpp"
#include <cstdint>

using namespace Numeric;
using namespace chassis;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

TX_THREAD PendulumThread;
TX_SEMAPHORE PendulumThreadSem;
uint8_t PendulumThreadStack[6144] = {0};
extern TX_SEMAPHORE IMUThreadSem;
extern TX_SEMAPHORE MotorAlive;

#ifdef DEBUG
struct pendulum_debug_t
{
    float alphal_eq;
    float alphar_eq;
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
    float llenref;
    float rlenref;
    float pitch;
    float pitch_dot;
    float Cl;
    float Cr;
    float Twl;
    float Twr;
    float Tpl;
    float Tpr;
    float Fsl;
    float Fsr;
    float Fl;
    float Fr;
    float Nl;
    float Nr;
    float N;
    float az;
    float llendot;
    float rlendot;
    float yaw;
    float yawref;
    float yaw_dot;
    float yaw_out;
    float lphi;
    float rphi;
    bool lflat;
    bool rflat;
    bool lneutral;
    bool rneutral;
    bool offground;
    bool landing;
    float lwheel_tor;
    float rwheel_tor;
    float ljoint4_tor;
    float ljoint1_tor;
    float rjoint4_tor;
    float rjoint1_tor;
    float ljoint4_pos;
    float ljoint1_pos;
    float rjoint4_pos;
    float rjoint1_pos;
    float lwheel_spd;
    float rwheel_spd;
    uint8_t stage;
    uint8_t state;
    uint8_t motorL4error;
    bool recoverd;
    float Tphil;
    float Tphir;
    float thread_time;
};
struct pid_tuning_t 
{
    float kp;
    float ki;
    float kd;
};
msg_ins_t debug_ins;
pendulum_debug_t pendulum_debug;
pid_tuning_t lenpd_tuning = {3200.0f, 0.0f, -700.0f};
pid_tuning_t rollpd_tuning = {0.5f, 0.00f, 0.0f};
DMMotorHandler *dmmotorhandler = DMMotorHandler::Instance();
#endif

[[noreturn]] void PendulumThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    /* motors initialization */
    M3508 RWheel;
    M3508 LWheel;

    DM8009P RJoint4;
    DM8009P RJoint1;
    DM8009P LJoint4;
    DM8009P LJoint1;

    DJIMotorHandler::Instance()->registerMotor(&LWheel, &hfdcan2, 0x202);
    LWheel.currentSet = 0;
    LWheel.gearBox = GearBox_XRoll;
    DJIMotorHandler::Instance()->registerMotor(&RWheel, &hfdcan2, 0x201);
    RWheel.currentSet = 0;
    RWheel.gearBox = GearBox_XRoll;

    DMMotorHandler::Instance()->registerMotor(&LJoint4, &hfdcan1, 0x02);
    LJoint4.controlMode = DMMotor::MIT_MODE;
    LJoint4.canType = DMMotor::DM_FDCAN;
    LJoint4.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor_Block(&LJoint4);

    DMMotorHandler::Instance()->registerMotor(&LJoint1, &hfdcan1, 0x01);
    LJoint1.controlMode = DMMotor::MIT_MODE;
    LJoint1.canType = DMMotor::DM_FDCAN;
    LJoint1.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor_Block(&LJoint1);

    DMMotorHandler::Instance()->registerMotor(&RJoint4, &hfdcan1, 0x03);
    RJoint4.controlMode = DMMotor::MIT_MODE;
    RJoint4.canType = DMMotor::DM_FDCAN;
    RJoint4.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor_Block(&RJoint4);

    DMMotorHandler::Instance()->registerMotor(&RJoint1, &hfdcan1, 0x04);
    RJoint1.controlMode = DMMotor::MIT_MODE;
    RJoint1.canType = DMMotor::DM_FDCAN;
    RJoint1.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor_Block(&RJoint1);

    /* If need to save zero points */
    // DMMotorHandler::Instance()->SaveZeroPosition(&LJoint4);
    // DMMotorHandler::Instance()->SaveZeroPosition(&RJoint4);
    // DMMotorHandler::Instance()->SaveZeroPosition(&LJoint1);
    // DMMotorHandler::Instance()->SaveZeroPosition(&RJoint1);

    /* one message initialization */
    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
    msg_cmd_t cmd{};
    
    om_topic_t *pendulum_pub = om_config_topic(nullptr, "ca", "pendulum", sizeof(msg_pendulum_t));
    msg_pendulum_t pendulum_data{};
    pendulum_data.reset_len = Lmin;

    /* solvers */
    Pendulum<DM8009P, M3508> lpendulum(false, &LJoint1, &LJoint4, &LWheel);
    Pendulum<DM8009P, M3508> rpendulum(true, &RJoint1, &RJoint4, &RWheel);

    /* odom */
    Odometry odom;

    /* lqr */
    float observedX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float refX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    LQR lqr(&refX[0], &observedX[0]);

    /* roll */
    PID roll_pd(0.7f, 0.0001f, 1.4f, 0.05f, 0.005f);

    /* state machine */
    chassis_state_e chassis_state = RELAX;
    jump_stage_e jump_stage = DONT;

    /* force&torque */
    float Twl = 0.0f, Twr = 0.0f;
    float Fl[2] = {0.0f, 0.0f}, Fr[2] = {0.0f, 0.0f};

    /* counters */
    uint32_t recover_cnt = 0, jumpair_cnt = 0;
    uint8_t check_cnt = 0;
    uint32_t dead_cnt = 0, alive_cnt = 0;
    uint32_t airprotect_cnt = 0, flipover_cnt = 0;

    /* flags */
    bool offground = false, landing = false;
    bool pre_stair = false, first_jump = false;

    /* helper */
    float maintain_yaw = 0.0f;

    float last_thread_time = 0.0f;
    float cur_thread_time = 0.0f;

    for (;;)
    {
        cur_thread_time = DWT_GetTimeline_us();
        pendulum_debug.thread_time = cur_thread_time - last_thread_time;
        last_thread_time = cur_thread_time;

        om_suber_export(ins_suber, &ins, false);
        om_suber_export(cmd_suber, &cmd, false);

        float pitch = ins.pitch*DegreeToRad;
        float dpitch = ins.gyro_p;
        float yaw = ins.total_yaw*DegreeToRad;
        float dyaw = ins.gyro_y; 
        float vlwheel = -LWheel.motorFeedback.speedFdb * Rwheel;
        float vrwheel = RWheel.motorFeedback.speedFdb * Rwheel;

        odom.Update(ins.quaternion, ins.accel, (vlwheel+vrwheel)*0.5f, yaw);
        lpendulum.Solve(pitch, dpitch, odom.az);
        rpendulum.Solve(pitch, dpitch, odom.az);

        /* [x, dx, yaw, dyaw, alphal, dalphal, alphar, dalphar, theta, dtheta] */
        observedX[0] = odom.x;
        observedX[1] = odom.v;
        observedX[2] = yaw;
        observedX[3] = dyaw;
        observedX[4] = lpendulum.alpha;
        observedX[5] = lpendulum.dalpha;
        observedX[6] = rpendulum.alpha;
        observedX[7] = rpendulum.dalpha;
        observedX[8] = pitch;
        observedX[9] = dpitch;

        float N = lpendulum.N+rpendulum.N;

        if (!cmd.move || tx_semaphore_get(&IMUThreadSem, TX_NO_WAIT) != TX_SUCCESS)
        {
            chassis_state = RELAX;
        }
        else
        {
            if (ins.accel[2] < 0.0f && chassis_state != RECOVER)
            {
                flipover_cnt++;
                if (flipover_cnt>1000)
                {
                    lpendulum.delta_init = false;
                    rpendulum.delta_init = false;
                    chassis_state = RELAX;
                    flipover_cnt = 0;
                }
            }
            else
                flipover_cnt = 0;
        }

        check_cnt ++;
        bool chassis_dead = dead_cnt > 500;
        bool chassis_alive = alive_cnt > 500;

        if (check_cnt == 3)
        {
            if ( LWheel.AliveCheck() != DJIMotor::MOTOR_ONLINE 
              || RWheel.AliveCheck() != DJIMotor::MOTOR_ONLINE)
            {
                dead_cnt++;
                alive_cnt = 0;
            }
            else 
            {
                alive_cnt++;
                dead_cnt = 0;
            }
            check_cnt = 0;

            if (chassis_dead) 
            {
                chassis_state = RELAX;
                DMMotorHandler::Instance()->DisableMotor(&LJoint1);
                DMMotorHandler::Instance()->DisableMotor(&LJoint4);
                DMMotorHandler::Instance()->DisableMotor(&RJoint1);
                DMMotorHandler::Instance()->DisableMotor(&RJoint4);
            }

            if (chassis_alive &&
                tx_semaphore_get(&MotorAlive, TX_NO_WAIT) != TX_SUCCESS)
            {
                DMMotorHandler::Instance()->EnableMotor(&LJoint1);
                DMMotorHandler::Instance()->EnableMotor(&LJoint4);
                DMMotorHandler::Instance()->EnableMotor(&RJoint1);
                DMMotorHandler::Instance()->EnableMotor(&RJoint4);
            }
        }

        switch (chassis_state) 
        {

        case RELAX:
        {
            lpendulum.Relax();
            rpendulum.Relax();
            odom.Reset();
            pendulum_data.ifresetlen = true;
            pendulum_data.recovered = false;
            pendulum_data.neutral = false;
            Fl[0] = 0.0f;
            Fl[1] = 0.0f;
            Fr[0] = 0.0f;
            Fr[1] = 0.0f;
            Twl = 0.0f;
            Twr = 0.0f;
            if (cmd.move && !chassis_dead)
            {
                if (ins.accel[2] < 0.0f)
                {
                    pendulum_data.recovered = false;
                    chassis_state = FLATTEN;
                }
                else
                    chassis_state = FLATTEN;
            }
                
            break;
        }

        case RECOVER:
        {
            odom.Reset();
            if (ins.accel[2] > 5.0f)
            {
                recover_cnt++;
            }
            else 
            {
                recover_cnt = 0;
            }

            if (recover_cnt >= 1000)
            {
                lpendulum.delta_init = false;
                rpendulum.delta_init = false;
                chassis_state = RELAX;
                break;
            }
            
            if (recover_cnt >= 100)
            {
                lpendulum.Relax();
                rpendulum.Relax();
            }
            else
            {
                if (!lpendulum.delta_init || !rpendulum.delta_init)
                {
                    if (pitch > 0.0f)
                    {
                        lpendulum.PhiControl(PI, 200.0f, 10.0f, 0.1f, true);
                        rpendulum.PhiControl(PI, 200.0f, 10.0f, 0.1f, true);
                    }
                    else
                    {
                        lpendulum.PhiControl(PI, 200.0f, 10.0f, 0.1f, true);
                        rpendulum.PhiControl(PI, 200.0f, 10.0f, 0.1f, true);
                    }
                }
            }
            
            break;
        }

        case FLATTEN:
        {
            
            pendulum_data.recovered = true;
            if (lpendulum.flat) 
            {
                Fl[0] = 0.0f;
                Fl[1] = 0.0f;
                lpendulum.Relax();
                lpendulum.delta_init = false;
                lpendulum.phi_updater.SetDefault(0.0f);
            }
            else
                Fl[1] = lpendulum.PhiControl(PI, 5.0f, 10.0f, 0.002f, false);
            
            if (rpendulum.flat)
            {
                Fr[0] = 0.0f;
                Fr[1] = 0.0f;
                rpendulum.Relax();
                rpendulum.delta_init = false;
                rpendulum.phi_updater.SetDefault(0.0f);
            } 
            else
                Fr[1] = rpendulum.PhiControl(PI, 5.0f, 10.0f, 0.002f, false);
            
            Fl[0] = 0.0f;
            Fr[0] = 0.0f;
            Twl = (cmd.v>0.0f)?1.5f:0.0f;
            Twr = (cmd.v>0.0f)?1.5f:0.0f;

            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);

            if (lpendulum.flat && rpendulum.flat)
            {
                odom.Reset();
                chassis_state = NEUTRAL;
            }
            break;
        }

        case NEUTRAL:
        {
            /* [x, dx, yaw, dyaw, alphal, dalphal, alphar, dalphar, theta, dtheta] */
            refX[0] = observedX[0];
            refX[1] = observedX[1];
            refX[2] = cmd.yaw;
            refX[3] = 0.0f;
            refX[4] = lpendulum.alpha_eq;
            refX[5] = 0.0f;
            refX[6] = rpendulum.alpha_eq;
            refX[7] = 0.0f;
            refX[8] = pitch_eq;
            refX[9] = 0.0f;
            lqr.lqr_type = LQR_STANDUP;
            lqr.Update(lpendulum.len, rpendulum.len);

            Twl = lqr.Tout[0];
            Twr = lqr.Tout[1];
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];

            Fl[0] = lpendulum.LenControl(lpendulum.len+(Lmin-lpendulum.len)*0.6f)-lpendulum.Fs;
            Fr[0] = rpendulum.LenControl(rpendulum.len+(Lmin-rpendulum.len)*0.6f)-rpendulum.Fs;

            if (lpendulum.len>0.20f || rpendulum.len>0.20f) 
            {
                Fl[1] = 0.0f;
                Fr[1] = 0.0f;
            }

            if (Numeric::abs(lpendulum.alpha) > 0.8f || Numeric::abs(rpendulum.alpha) > 0.8f) 
            {
                Twl=0.0f;
                Twr=0.0f;
            }

            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);

            if (lpendulum.neutral && rpendulum.neutral)
            {
                odom.Reset();
                pendulum_data.neutral = true;
                chassis_state = NORMAL;
                lpendulum.delta_init = false;
                rpendulum.delta_init = false;
            }
            break;
        }

        case NORMAL:
        {

            /* [x, dx, yaw, dyaw, alphal, dalphal, alphar, dalphar, theta, dtheta] */
            refX[0] = cmd.x;
            refX[1] = cmd.v;
            refX[2] = cmd.yaw;
            refX[3] = cmd.dyaw;
            refX[4] = lpendulum.alpha_eq;
            refX[5] = 0.0f;
            refX[6] = rpendulum.alpha_eq;
            refX[7] = 0.0f;
            refX[8] = pitch_eq;
            refX[9] = 0.0f;

            if ((lpendulum.len+rpendulum.len)*0.5f > Lswitch)
            {
                lqr.lqr_type = LQR_HIGH;
                lqr.Update(lpendulum.len, rpendulum.len);
            }
            else
            {
                lqr.lqr_type = LQR_LOW;
                if (cmd.v == 0.0f)
                {
                    lqr.Update(lpendulum.len, rpendulum.len);
                }
                else
                {
                    lqr.Update(lpendulum.len, rpendulum.len);
                }
            }

            Twl = lqr.Tout[0];
            Twr = lqr.Tout[1];
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];

            roll_pd.ref = 0.0f;
            roll_pd.fdb = ins.roll*DegreeToRad;
            roll_pd.UpdateResult(ins.gyro_r);

            Fl[0] = lpendulum.LenControl(cmd.len+roll_pd.result) + Gff - lpendulum.Fs;
            Fr[0] = rpendulum.LenControl(cmd.len-roll_pd.result) + Gff - rpendulum.Fs;

            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);
            
            pendulum_data.ifresetlen = false;

            airprotect_cnt ++;

            offground = (N<0.0f) && (lpendulum.Freal<-100.0f) && (rpendulum.Freal<-100.0f) && (airprotect_cnt>500);

            if (cmd.gostair && !pre_stair) 
            {
                lpendulum.delta_init = false;
                rpendulum.delta_init = false;
                chassis_state = GOSTAIR; 
            }

            if (cmd.prejump) 
            { 
                chassis_state = JUMP; 
                jump_stage = START;
                jumpair_cnt = 0;
            }

            if (cmd.spin)
            {
                chassis_state = SPIN;
            }

            if (offground) 
            {
                chassis_state = OFFGROUND;
            }

            pre_stair = cmd.gostair;

            lpendulum.len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
            rpendulum.len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
            break;
        }

        case OFFGROUND:
        {
            refX[0] = observedX[0];
            refX[1] = observedX[1];
            refX[2] = observedX[2];
            refX[3] = observedX[3];
            refX[4] = pitch;
            refX[5] = 0.0f;
            refX[6] = pitch;
            refX[7] = 0.0f;
            refX[8] = observedX[8];
            refX[9] = observedX[9];
            lqr.lqr_type = LQR_LOW;
            lqr.Update(lpendulum.len, rpendulum.len);

            // Twl = 0.0f;
            // Twr = 0.0f;
            Twl = lqr.Tout[0];
            Twr = lqr.Tout[1];
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];
            Fl[0] = lpendulum.LenControl(cmd.len);
            Fr[0] = rpendulum.LenControl(cmd.len);
            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);

            landing = (N>50.0f) && (lpendulum.Freal>-100.0f) && (rpendulum.Freal>-100.0f);
            if (landing) 
            {
                pendulum_data.ifresetlen = true;
                pendulum_data.reset_len = Lmin;
                airprotect_cnt = 0;
                chassis_state = NORMAL;
            }
            break;
        }

        case SPIN:
        {
            /* [x, dx, yaw, dyaw, alphal, dalphal, alphar, dalphar, theta, dtheta] */
            refX[0] = odom.x;//cmd.x;
            refX[1] = cmd.v;
            refX[2] = cmd.yaw;
            refX[3] = cmd.dyaw;
            refX[4] = 0.0f;//lpendulum.alpha_eq;
            refX[5] = 0.0f;
            refX[6] = 0.0f;//rpendulum.alpha_eq;
            refX[7] = 0.0f;
            refX[8] = 0.0f;//pitch_eq;
            refX[9] = 0.0f;

            lqr.lqr_type = LQR_LOW;
            lqr.Update(lpendulum.len, rpendulum.len);

            Twl = lqr.Tout[0];
            Twr = lqr.Tout[1];
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];

            Fl[0] = lpendulum.LenControl(Lmin)+Gff-lpendulum.Fs;
            Fr[0] = rpendulum.LenControl(Lmin)+Gff-rpendulum.Fs;

            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);

            if (!cmd.spin) 
            {
                // odom.Reset();
                chassis_state = NORMAL; 
            }

            lpendulum.len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
            rpendulum.len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
            break;
        }

        case GOSTAIR:
        {
            // odom.Reset();

            lpendulum.PhiControl(PI, 10.0f, 5.0f, 0.01f, false);
            rpendulum.PhiControl(PI, 10.0f, 5.0f, 0.01f, false);

            Twl = 0.0f;
            Twr = 0.0f;
            Fl[0] = 0.0f;
            Fl[1] = 0.0f;
            Fr[0] = 0.0f;
            Fr[1] = 0.0f;
            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);

            if (lpendulum.flat && rpendulum.flat)   
            {
                chassis_state = NEUTRAL;
                pendulum_data.ifresetlen = true;
                pendulum_data.reset_len = Lmin;
            }
            break;
        }

        case JUMP:
        {

            switch (jump_stage) 
            {
            case DONT:
            {
                chassis_state = NORMAL;
                break;
            }

            case START:
            {
                first_jump = true;

                /* [x, dx, yaw, dyaw, alphal, dalphal, alphar, dalphar, theta, dtheta] */
                refX[0] = cmd.x;
                refX[1] = cmd.v;
                refX[2] = cmd.yaw;
                refX[3] = cmd.dyaw;
                refX[4] = lpendulum.alpha_eq;
                refX[5] = 0.0f;
                refX[6] = rpendulum.alpha_eq;
                refX[7] = 0.0f;
                refX[8] = 0.0f;
                refX[9] = 0.0f;

                lqr.lqr_type = LQR_LOW;
                lqr.Update(lpendulum.len, rpendulum.len);

                Twl = lqr.Tout[0];
                Twr = lqr.Tout[1];
                Fl[1] = lqr.Tout[2];
                Fr[1] = lqr.Tout[3];

                roll_pd.ref = 0.0f;
                roll_pd.fdb = ins.roll*DegreeToRad;
                roll_pd.UpdateResult(ins.gyro_r);

                Fl[0] = lpendulum.LenControl(Lmin+roll_pd.result);
                Fr[0] = rpendulum.LenControl(Lmin-roll_pd.result);

                if (cmd.ifjump)
                {
                    jump_stage = EXTENDING;
                }

                break;
            }

            case EXTENDING:
            {
                if (first_jump)
                    maintain_yaw = yaw;

                refX[0] = observedX[0];
                refX[1] = observedX[1];
                refX[2] = maintain_yaw;
                refX[3] = 0.0f;
                refX[4] = lpendulum.alpha_eq;
                refX[5] = 0.0f;
                refX[6] = rpendulum.alpha_eq;
                refX[7] = 0.0f;
                refX[8] = 0.0f;
                refX[9] = 0.0f;

                lqr.lqr_type = LQR_LOW;
                lqr.Update(lpendulum.len, rpendulum.len);

                Twl = lqr.Tout[0];
                Twr = lqr.Tout[1];
                Fl[1] = lqr.Tout[2];
                Fr[1] = lqr.Tout[3];
                Fl[0] = 300.0f;
                Fr[0] = 300.0f;

                if (lpendulum.len>=0.29f && rpendulum.len>=0.29f)
                {
                    jumpair_cnt = 0;
                    jump_stage = INAIR;
                }

                break;
            }

            case INAIR:
            {
                // odom.Reset();

                jumpair_cnt++;

                refX[0] = observedX[0];
                refX[1] = observedX[1];
                refX[2] = observedX[2];
                refX[3] = observedX[3];
                refX[4] = pitch;
                refX[5] = 0.0f;
                refX[6] = pitch;
                refX[7] = 0.0f;
                refX[8] = observedX[8];
                refX[9] = observedX[9];
                lqr.lqr_type = LQR_LOW;
                lqr.Update(lpendulum.len, rpendulum.len);

                Twl = 0.0f;
                Twr = 0.0f;
                Fl[1] = lqr.Tout[2];
                Fr[1] = lqr.Tout[3];
                Fl[0] = lpendulum.LenControl(Lmin);
                Fr[0] = rpendulum.LenControl(Lmin);
                lpendulum.TorqueControl(Fl, Twl);
                rpendulum.TorqueControl(Fr, Twr);

                if (jumpair_cnt > 150)
                {
                    jump_stage = LANDING;
                }

                break;
            }

            case LANDING:
            {

                // odom.Reset();

                refX[0] = observedX[0];
                refX[1] = observedX[1];
                refX[2] = observedX[2];
                refX[3] = observedX[3];
                refX[4] = pitch;
                refX[5] = 0.0f;
                refX[6] = pitch;
                refX[7] = 0.0f;
                refX[8] = observedX[8];
                refX[9] = observedX[9];
                lqr.lqr_type = LQR_LOW;
                lqr.Update(lpendulum.len, rpendulum.len);

                Twl = 0.0f;
                Twr = 0.0f;
                Fl[1] = lqr.Tout[2];
                Fr[1] = lqr.Tout[3];
                Fl[0] = lpendulum.LenControl(Lmin);
                Fr[0] = rpendulum.LenControl(Lmin);
                lpendulum.TorqueControl(Fl, Twl);
                rpendulum.TorqueControl(Fr, Twr);

                if (ins.accel[2] > 40.0f)
                {
                    // airland_cnt = 0;
                    pendulum_data.ifresetlen = true;
                    jump_stage = BACK;
                }
                break;
            }

            case BACK:
            {
                /* [x, dx, yaw, dyaw, alphal, dalphal, alphar, dalphar, theta, dtheta] */
                refX[0] = observedX[0]+0.0015f;
                refX[1] = observedX[1]+0.15f;
                refX[2] = maintain_yaw;
                refX[3] = 0.0f;
                refX[4] = lpendulum.alpha_eq;
                refX[5] = 0.0f;
                refX[6] = rpendulum.alpha_eq;
                refX[7] = 0.0f;
                refX[8] = 0.0f;
                refX[9] = 0.0f;

                lqr.lqr_type = LQR_LOW;
                lqr.Update(lpendulum.len, rpendulum.len);

                Twl = lqr.Tout[0];
                Twr = lqr.Tout[1];
                Fl[1] = lqr.Tout[2];
                Fr[1] = lqr.Tout[3];

                roll_pd.ref = 0.0f;
                roll_pd.fdb = ins.roll*DegreeToRad;
                roll_pd.UpdateResult(ins.gyro_r);

                Fl[0] = lpendulum.LenControl(Lmax+roll_pd.result);
                Fr[0] = rpendulum.LenControl(Lmax-roll_pd.result);

                lpendulum.TorqueControl(Fl, Twl);
                rpendulum.TorqueControl(Fr, Twr);
                
                pendulum_data.ifresetlen = false;

                // airland_cnt ++;
                
                if (first_jump)// && airland_cnt > 800)
                {
                    first_jump = false;
                    chassis_state = GOSTAIR;
                    // airland_cnt = 0;
                }
                break;
            }

            }
            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);
            break;
        }

        default:
            break;
        }

        DMMotorHandler::Instance()->sendControlData();
        DJIMotorHandler::Instance()->sendControlData();

        pendulum_data.x = odom.x;
        pendulum_data.v = odom.v;
        pendulum_data.N = (lpendulum.N+rpendulum.N)*0.5f;
        pendulum_data.len = (lpendulum.len+rpendulum.len)*0.5f;
        om_publish(pendulum_pub, &pendulum_data, sizeof(msg_pendulum_t), true, false);  
        tx_semaphore_put(&PendulumThreadSem);

    #ifdef DEBUG
        debug_ins = ins;

        pendulum_debug.ljoint1_pos = LJoint1.motorFeedback.positionFdb;
        pendulum_debug.ljoint4_pos = LJoint4.motorFeedback.positionFdb;
        pendulum_debug.rjoint1_pos = RJoint1.motorFeedback.positionFdb;
        pendulum_debug.rjoint4_pos = RJoint4.motorFeedback.positionFdb;
        pendulum_debug.ljoint1_tor = LJoint1.motorFeedback.torqueFdb;
        pendulum_debug.ljoint4_tor = LJoint4.motorFeedback.torqueFdb;
        pendulum_debug.rjoint1_tor = RJoint1.motorFeedback.torqueFdb;
        pendulum_debug.rjoint4_tor = RJoint4.motorFeedback.torqueFdb;
        pendulum_debug.lwheel_tor = LWheel.motorFeedback.currentFdb/1400.0f;
        pendulum_debug.rwheel_tor = RWheel.motorFeedback.currentFdb/1400.0f;
        pendulum_debug.lwheel_spd = vlwheel;
        pendulum_debug.rwheel_spd = vrwheel;
        
        pendulum_debug.x = odom.x;
        pendulum_debug.xref = cmd.x;
        pendulum_debug.v = odom.v;
        pendulum_debug.vref = cmd.v;
        pendulum_debug.l_ref = cmd.len;
        pendulum_debug.llenref = lpendulum.len_pd.ref;
        pendulum_debug.rlenref = rpendulum.len_pd.ref;
        pendulum_debug.llen = lpendulum.len;
        pendulum_debug.rlen = rpendulum.len;
        pendulum_debug.lphi = lpendulum.phi;
        pendulum_debug.rphi = rpendulum.phi;
        pendulum_debug.alphal = lpendulum.alpha;
        pendulum_debug.alphar = rpendulum.alpha;
        pendulum_debug.alphal_eq = lpendulum.alpha_eq;
        pendulum_debug.alphar_eq = rpendulum.alpha_eq;
        pendulum_debug.alphal_dot = lpendulum.dalpha;
        pendulum_debug.alphar_dot = rpendulum.dalpha;
        pendulum_debug.pitch = pitch;
        pendulum_debug.pitch_dot = dpitch;
        pendulum_debug.yaw = yaw;
        pendulum_debug.yawref = cmd.yaw;
        pendulum_debug.yaw_dot = cmd.dyaw;
        pendulum_debug.yaw_out = dyaw;
        pendulum_debug.Twl = Twl;
        pendulum_debug.Twr = Twr;
        pendulum_debug.Tpl = lqr.Tout[2];
        pendulum_debug.Tpr = lqr.Tout[3];
        pendulum_debug.Fsl = lpendulum.Fs;
        pendulum_debug.Fsr = rpendulum.Fs;
        pendulum_debug.Fl = lpendulum.Freal;
        pendulum_debug.Fr = rpendulum.Freal;
        pendulum_debug.offground = offground;
        pendulum_debug.landing = landing;
        pendulum_debug.lneutral = lpendulum.neutral;
        pendulum_debug.rneutral = rpendulum.neutral;
        pendulum_debug.lflat = lpendulum.flat;
        pendulum_debug.rflat = rpendulum.flat;
        pendulum_debug.Nl = lpendulum.N;
        pendulum_debug.Nr = rpendulum.N;
        pendulum_debug.N = N;
        pendulum_debug.az = odom.az;
        pendulum_debug.stage = jump_stage;
        pendulum_debug.lphi = lpendulum.phi;
        pendulum_debug.rphi = rpendulum.phi;
        pendulum_debug.Tphil = lpendulum.phi_pd.result;
        pendulum_debug.Tphir = rpendulum.phi_pd.result;
        roll_pd.Tuning(rollpd_tuning.kp, rollpd_tuning.ki, rollpd_tuning.kd);
        pendulum_debug.motorL4error = RJoint4.motorFeedback.ERR;
        pendulum_debug.state = chassis_state;
    #endif

        tx_thread_sleep(1);
    }
}
