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
    float Fl;
    float Fr;
    float Nl;
    float Nr;
    float N;
    float llendot;
    float rlendot;
    float yaw;
    float yawref;
    float yaw_dot;
    float yaw_out;
    bool lflat;
    bool rflat;
    bool lneutral;
    bool rneutral;
    bool lair;
    bool rair;
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
    float simple_odom_x;
    float simple_odom_v;

};
struct pid_tuning_t 
{
    float kp;
    float ki;
    float kd;
};
msg_ins_t debug_ins;
pendulum_debug_t pendulum_debug;
pid_tuning_t lenpd_tuning = {6000.0f, 0.0f, -900.0f};
pid_tuning_t rollpd_tuning = {0.7f, 0.00f, -0.1f};
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

    DJIMotorHandler::Instance()->registerMotor(&LWheel, &hfdcan2, 0x201);
    LWheel.currentSet = 0;
    LWheel.gearBox = GearBox_XRoll;
    DJIMotorHandler::Instance()->registerMotor(&RWheel, &hfdcan2, 0x202);
    RWheel.currentSet = 0;
    RWheel.gearBox = GearBox_XRoll;

    DMMotorHandler::Instance()->registerMotor(&LJoint4, &hfdcan1, 0x03);
    LJoint4.controlMode = DMMotor::MIT_MODE;
    LJoint4.canType = DMMotor::DM_FDCAN;
    LJoint4.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor_Block(&LJoint4);

    DMMotorHandler::Instance()->registerMotor(&LJoint1, &hfdcan1, 0x02);
    LJoint1.controlMode = DMMotor::MIT_MODE;
    LJoint1.canType = DMMotor::DM_FDCAN;
    LJoint1.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor_Block(&LJoint1);

    DMMotorHandler::Instance()->registerMotor(&RJoint4, &hfdcan1, 0x04);
    RJoint4.controlMode = DMMotor::MIT_MODE;
    RJoint4.canType = DMMotor::DM_FDCAN;
    RJoint4.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor_Block(&RJoint4);
    // DMMotorHandler::Instance()->DisableMotor(&RJoint4);

    DMMotorHandler::Instance()->registerMotor(&RJoint1, &hfdcan1, 0x01);
    RJoint1.controlMode = DMMotor::MIT_MODE;
    RJoint1.canType = DMMotor::DM_FDCAN;
    RJoint1.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor_Block(&RJoint1);

    /* If need to save zero points */
    // DMMotorHandler::Instance()->SaveZeroPosition(&LJoint4);
    // DMMotorHandler::Instance()->SaveZeroPosition(&LJoint1);
    // DMMotorHandler::Instance()->SaveZeroPosition(&RJoint4);
    // DMMotorHandler::Instance()->SaveZeroPosition(&RJoint1);

    /* one message initialization */
    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
    msg_cmd_t cmd{};
    
    om_topic_t *pendulum_pub = om_config_topic(nullptr, "ca", "pendulum", sizeof(msg_pendulum_t));
    msg_pendulum_t pendulum_data{};

    /* solvers */
    Pendulum<DM8009P, M3508> lpendulum(false, &LJoint1, &LJoint4, &LWheel);
    Pendulum<DM8009P, M3508> rpendulum(true, &RJoint1, &RJoint4, &RWheel);
    /* odom */
    Odometry odom;
    SimpleOdom simple_odom;
    /* lqr */
    float observedX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float refX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    LQR lqr(&refX[0], &observedX[0]);
    /* roll */
    PID roll_pd(0.7f, 0.0001f, 1.4f, 3.0f, 0.005f);
    /* state machine */
    chassis_state_e chassis_state = RELAX;
    jump_stage_e jump_stage = DONT;

    /* force&torque */
    float Twl = 0.0f;
    float Twr = 0.0f;
    float Fl[2] = {0.0f, 0.0f};
    float Fr[2] = {0.0f, 0.0f};

    SLOPE recovery_updater(0.0f, 0.01f);

    bool pre_stair = false;

    uint16_t recover_count = 0;
    uint32_t airland_cnt = 0;
    uint16_t jumpair_cnt = 0;
    bool first_jump = false;
    float maintain_yaw = 0.0f;

    uint8_t check_cnt = 0;
    uint32_t dead_cnt = 0;
    uint32_t alive_cnt = 0;

    for (;;)
    {
        float thread_start_time = tx_time_get();
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(cmd_suber, &cmd, false);

        float pitch = ins.pitch*DegreeToRad;
        float dpitch = ins.gyro_p;
        float yaw = ins.total_yaw*DegreeToRad;
        float dyaw = ins.gyro_y;
        float vlwheel = -LWheel.motorFeedback.speedFdb * Rwheel;
        float vrwheel = RWheel.motorFeedback.speedFdb * Rwheel;

        odom.Update(ins.quaternion, ins.accel,(vlwheel+vrwheel)*0.5f, yaw);
        simple_odom.Update((vlwheel+vrwheel)*0.5f);
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
            chassis_state = RELAX;

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
            pendulum_data.reset_len = true;
            pendulum_data.recovered = true;
            pendulum_data.neutral = false;
            if (cmd.move && !chassis_dead)
            {
                if (ins.accel[2] < 0.0f)
                {
                    pendulum_data.recovered = false;
                    chassis_state = RECOVER;
                }
                else
                    chassis_state = FLATTEN;
            }
                
            break;
        }

        case RECOVER:
        {
            if (ins.accel[2] > 5.0f)
            {
                recover_count++;
            }
            else 
            {
                recover_count = 0;
            }

            if (recover_count >= 1000)
            {
                lpendulum.delta_init = false;
                rpendulum.delta_init = false;
                chassis_state = RELAX;
                break;
            }
            
            lpendulum.DeltaPhiControl(PI, 0.7f, 1.4f);
            rpendulum.DeltaPhiControl(PI, 0.7f, 1.4f);
            break;
        }

        case FLATTEN:
        {
            odom.Reset();
            if (lpendulum.flat) 
                lpendulum.Relax();
            else
                lpendulum.DeltaPhiControl(PI, 1.5f, 12.0f);
            
            if (rpendulum.flat)
                rpendulum.Relax();
            else
                rpendulum.DeltaPhiControl(PI, 1.5f, 12.0f);
            if (lpendulum.flat && rpendulum.flat)
                chassis_state = NEUTRAL;
            break;
        }

        case GOSTAIR:
        {
            odom.Reset();

            lpendulum.DeltaPhiControl(PI, 2.0f, 6.0f);
            rpendulum.DeltaPhiControl(PI, 2.0f, 6.0f);

            if (lpendulum.phi < 1.7f && lpendulum.phi > 0.0f)
                Twl = 0.1f;
            else
                Twl = 0.0f;
            if (rpendulum.phi < 1.7f && rpendulum.phi > 0.0f)
                Twr = 0.1f;
            else
                Twr = 0.0f;
            Fl[0] = 0.0f;
            Fl[1] = 0.0f;
            Fr[0] = 0.0f;
            Fr[1] = 0.0f;
            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);

            if (((lpendulum.phi>-3.14f&&lpendulum.phi<-2.0f) 
                && (rpendulum.phi>-3.14f&&rpendulum.phi<-2.0f))
                || ((lpendulum.phi>=2.4f&&lpendulum.phi<=3.1f)
                && (rpendulum.phi>=2.4f&&rpendulum.phi<=3.1f))
                )   
            {
                chassis_state = NEUTRAL;
                pendulum_data.reset_len = true;
            }
            break;
        }

        case NEUTRAL:
        {
            /* [x, dx, yaw, dyaw, alphal, dalphal, alphar, dalphar, theta, dtheta] */
            refX[0] = cmd.x;
            refX[1] = cmd.v;
            refX[2] = cmd.yaw;
            refX[3] = cmd.dyaw;
            refX[4] = 0.0f;
            refX[5] = 0.0f;
            refX[6] = 0.0f;
            refX[7] = 0.0f;
            refX[8] = 0.0f;
            refX[9] = 0.0f;
            lqr.lqr_type = LQR_STANDUP;
            lqr.Update(lpendulum.len, rpendulum.len, false);

            Twl = lqr.Tout[0];
            Twr = lqr.Tout[1];
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];
            Fl[0] = lpendulum.LenControl(Lmin);
            Fr[0] = rpendulum.LenControl(Lmin);

            if (lpendulum.len>0.19f || rpendulum.len>0.19f) 
            {
                Fl[1]=0.0f;
                Fr[1]=0.0f;
            }

            if (Numeric::abs(lpendulum.alpha) > 0.5f || Numeric::abs(rpendulum.alpha) > 0.5f) 
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
            refX[8] = 0.005f;
            refX[9] = 0.0f;

            if ((lpendulum.len+rpendulum.len)*0.5f > 0.23f)
            {
                lqr.lqr_type = LQR_HIGH;
                lqr.Update(lpendulum.len, rpendulum.len, false);
            }
            else
            {
                lqr.lqr_type = LQR_LOW;
                if (cmd.v == 0.0f)
                {
                    lqr.Update(lpendulum.len, rpendulum.len, true);
                }
                else
                {
                    lqr.Update(lpendulum.len, rpendulum.len, false);
                }
            }

            Twl = lqr.Tout[0];
            Twr = lqr.Tout[1];
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];

            roll_pd.ref = 0.0f;
            roll_pd.fdb = ins.roll*DegreeToRad;
            roll_pd.UpdateResult(ins.gyro_r);

            Fl[0] = lpendulum.LenControl(cmd.len+roll_pd.result);
            Fr[0] = rpendulum.LenControl(cmd.len-roll_pd.result);

            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);
            
            pendulum_data.reset_len = false;

            airland_cnt ++;
            
            if (lpendulum.airborne && rpendulum.airborne && airland_cnt > 600) 
            {
                chassis_state = OFFGROUND; 
            }

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

            pre_stair = cmd.gostair;
            break;
        }

        case SPIN:
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
            refX[8] = 0.0f;
            refX[9] = 0.0f;

            // observedX[0] = refX[0];
            // observedX[1] = simple_odom.v;

            lqr.lqr_type = LQR_SPIN;
            lqr.Update(lpendulum.len, rpendulum.len, true);

            Twl = lqr.Tout[0];
            Twr = lqr.Tout[1];
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];

            Fl[0] = lpendulum.LenControl(Lmin);
            Fr[0] = rpendulum.LenControl(Lmin);

            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);

            if (!cmd.spin) 
            {
                odom.Reset();
                chassis_state = NORMAL; 
            }
            break;
        }

        case OFFGROUND:
        {
            odom.Reset();

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
            lqr.Update(lpendulum.len, rpendulum.len,false);

            Twl = 0.0f;
            Twr = 0.0f;
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];
            Fl[0] = lpendulum.LenControl(Lmax);
            Fr[0] = rpendulum.LenControl(Lmax);
            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);

            if ((!lpendulum.airborne && !rpendulum.airborne) || ins.accel[2] > 20.0f)
            {
                airland_cnt = 0;
                pendulum_data.reset_len = true;
                chassis_state = NORMAL; 
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
                lqr.Update(lpendulum.len, rpendulum.len, false);

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
                lqr.Update(lpendulum.len, rpendulum.len, false);

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
                odom.Reset();

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
                lqr.Update(lpendulum.len, rpendulum.len, false);

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

                odom.Reset();

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
                lqr.Update(lpendulum.len, rpendulum.len,false);

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
                    airland_cnt = 0;
                    pendulum_data.reset_len = true;
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
                lqr.Update(lpendulum.len, rpendulum.len, false);

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
                
                pendulum_data.reset_len = false;

                airland_cnt ++;
                
                if (first_jump && airland_cnt > 800)
                {
                    first_jump = false;
                    chassis_state = GOSTAIR;
                    airland_cnt = 0;
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
        pendulum_debug.Fl = Fl[0];
        pendulum_debug.Fr = Fr[0];
        pendulum_debug.lair = lpendulum.airborne;
        pendulum_debug.rair = rpendulum.airborne;
        pendulum_debug.lneutral = lpendulum.neutral;
        pendulum_debug.rneutral = rpendulum.neutral;
        pendulum_debug.lflat = lpendulum.flat;
        pendulum_debug.rflat = rpendulum.flat;
        pendulum_debug.Nl = lpendulum.N;
        pendulum_debug.Nr = rpendulum.N;
        pendulum_debug.N = N;
        pendulum_debug.stage = jump_stage;
        pendulum_debug.simple_odom_v = simple_odom.v;
        pendulum_debug.simple_odom_x = simple_odom.x;
        lpendulum.len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
        rpendulum.len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
        roll_pd.Tuning(rollpd_tuning.kp, rollpd_tuning.ki, rollpd_tuning.kd);
        pendulum_debug.motorL4error = RJoint4.motorFeedback.ERR;
        pendulum_debug.state = chassis_state;
    #endif

        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));
    }
}
