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

using namespace Numeric;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

TX_THREAD PendulumThread;
TX_SEMAPHORE PendulumThreadSem;
uint8_t PendulumThreadStack[8192] = {0};
extern TX_SEMAPHORE IMUThreadSem;

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
    float Flreal;
    float Frreal;
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
    bool offground;
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
};
struct pid_tuning_t 
{
    float kp;
    float ki;
    float kd;
};
msg_ins_t debug_ins;
pendulum_debug_t pendulum_debug;
pid_tuning_t lenpd_tuning = {2000.0f, 0.0f, -500.0f};
pid_tuning_t rollpd_tuning = {0.5f, 0.0f, -0.5f};
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

    DMMotorHandler::Instance()->registerMotor(&LJoint1, &hfdcan1, 0x04);
    LJoint1.controlMode = DMMotor::MIT_MODE;
    LJoint1.canType = DMMotor::DM_FDCAN;
    LJoint1.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor_Block(&LJoint1);

    DMMotorHandler::Instance()->registerMotor(&RJoint4, &hfdcan1, 0x01);
    RJoint4.controlMode = DMMotor::MIT_MODE;
    RJoint4.canType = DMMotor::DM_FDCAN;
    RJoint4.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor_Block(&RJoint4);

    DMMotorHandler::Instance()->registerMotor(&RJoint1, &hfdcan1, 0x02);
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
    /* lqr */
    float observedX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float refX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    LQR lqr(&refX[0], &observedX[0]);
    /* len */
    PID rleg_len_pd(6000.0f, 0.0f, -1000.0f, 125.0f, 0.0f, PID_DVEL);
    PID lleg_len_pd(6000.0f, 0.0f, -1000.0f, 125.0f, 0.0f, PID_DVEL);
    /* roll */
    PID roll_pd(0.7f, 0.0f, 1.4f, 3.0f, 0.0f);
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

    for (;;)
    {
        float thread_start_time = tx_time_get();
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(cmd_suber, &cmd, false);

        float pitch = ins.pitch*DegreeToRad;
        float dpitch = ins.gyro_p;
        float yaw = ins.total_yaw*DegreeToRad;
        float dyaw = ins.gyro_y;
        float vlwheel = -LWheel.motorFeedback.speedFdb * WHEEL_RADIUS;
        float vrwheel = RWheel.motorFeedback.speedFdb * WHEEL_RADIUS;

        odom.Update(ins.quaternion, ins.accel,(vlwheel+vrwheel)*0.5f, yaw);
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

        switch (chassis_state) 
        {

        case RELAX:

            lpendulum.Relax();
            rpendulum.Relax();
            odom.Reset();
            pendulum_data.neutral = false;
            if (cmd.move)
            {
                if (ins.accel[2] < 0.0f)
                    chassis_state = RECOVER;
                else
                    chassis_state = FLATTEN;
            }
                
            break;

        case RECOVER:

            if (ins.accel[2] > 0.0f)
            {
                recover_count++;
            }

            if (recover_count >= 100)
            {
                lpendulum.delta_init = false;
                rpendulum.delta_init = false;
                chassis_state = FLATTEN;
                break;
            }
            
            // lpendulum.DeltaPControl(-recovery_updater.UpdateVal(JOINT_RECOVER_DELTA), 1.5f, 1.0f);
            // rpendulum.DeltaPControl(-recovery_updater.UpdateVal(JOINT_RECOVER_DELTA), 1.5f, 1.0f);
            lpendulum.SpdControl(-recovery_updater.UpdateVal(0.5f), 1.0f);
            rpendulum.SpdControl(-recovery_updater.UpdateVal(0.5f), 1.0f);
            break;

        case FLATTEN:

            odom.Reset();
            if (lpendulum.flat) 
                lpendulum.Relax();
            else
                lpendulum.DeltaPControl(-JOINT_FLAT_DELTA, 2.0f, 10.0f);
            
            if (rpendulum.flat)
                rpendulum.Relax();
            else
                rpendulum.DeltaPControl(-JOINT_FLAT_DELTA, 2.0f, 10.0f);
            if (lpendulum.flat && rpendulum.flat)
                chassis_state = NEUTRAL;
            break;

        case GOSTAIR:
        
            odom.Reset();

            lpendulum.DeltaPControl(JOINT_STAIR_DELTA, 2.0f, 6.0f);
            rpendulum.DeltaPControl(JOINT_STAIR_DELTA, 2.0f, 6.0f);

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

            if ((lpendulum.phi>-3.14f&&lpendulum.phi<-2.5f) 
                && (rpendulum.phi>-3.14f&&rpendulum.phi<-2.5f) )   
            {
                chassis_state = NEUTRAL;
            }
            break;

        case NEUTRAL:

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
            lqr.lqr_type = LQR_LOW;
            lqr.Update(lpendulum.len, rpendulum.len, false);

            Twl = lqr.Tout[0];
            Twr = lqr.Tout[1];
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];

            lleg_len_pd.ref = MIN_LEG_LEN;
            rleg_len_pd.ref = MIN_LEG_LEN;

            lleg_len_pd.fdb = lpendulum.len;
            lleg_len_pd.UpdateResult(lpendulum.dlen);
            Fl[0] = lleg_len_pd.result - GRAVITY_FF;
            
            rleg_len_pd.fdb = rpendulum.len;
            rleg_len_pd.UpdateResult(rpendulum.dlen);
            Fr[0] = rleg_len_pd.result - GRAVITY_FF;

            if (lpendulum.len>0.18f || rpendulum.len>0.18f) 
            {
                Fl[1]=0.0f;
                Fr[1]=0.0f;
            }

            if (Numeric::abs(lpendulum.alpha) > 0.7f || Numeric::abs(rpendulum.alpha) > 0.7f) 
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

        case NORMAL:

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

            lleg_len_pd.ref = cmd.len+roll_pd.result;//-0.03f
            rleg_len_pd.ref = cmd.len-roll_pd.result;//-0.03f
            lleg_len_pd.fdb = lpendulum.len;
            lleg_len_pd.UpdateResult(lpendulum.dlen);
            Fl[0] = lleg_len_pd.result - GRAVITY_FF;
            rleg_len_pd.fdb = rpendulum.len;
            rleg_len_pd.UpdateResult(rpendulum.dlen);
            Fr[0] = rleg_len_pd.result - GRAVITY_FF;

            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);
            
            pendulum_data.reset_len = false;
            if (Fl[0] < -120.0f && Fr[0] < -120.0f) 
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
            }
            pre_stair = cmd.gostair;
            break;

        case SPIN:

            refX[0] = observedX[0];
            refX[1] = observedX[1];
            refX[2] = cmd.yaw;
            refX[3] = cmd.dyaw;
            refX[4] = 0.01f;
            refX[5] = 0.0f;
            refX[6] = 0.01f;
            refX[7] = 0.0f;
            refX[8] = 0.0f;
            refX[9] = 0.0f;

            lqr.lqr_type = LQR_LOW;
            lqr.Update(lpendulum.len, rpendulum.len, true);

            Twl = lqr.Tout[0];
            Twr = lqr.Tout[1];
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];

            lleg_len_pd.ref = MIN_LEG_LEN;
            rleg_len_pd.ref = MIN_LEG_LEN;
            lleg_len_pd.fdb = lpendulum.len;
            lleg_len_pd.UpdateResult(lpendulum.dlen);
            Fl[0] = lleg_len_pd.result - GRAVITY_FF;
            rleg_len_pd.fdb = rpendulum.len;
            rleg_len_pd.UpdateResult(rpendulum.dlen);
            Fr[0] = rleg_len_pd.result - GRAVITY_FF;

            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);

            if (!cmd.spin) { chassis_state = NORMAL; }
            break;

        case OFFGROUND:

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
            lleg_len_pd.ref = 0.30f;
            rleg_len_pd.ref = 0.30f;
            lleg_len_pd.fdb = lpendulum.len;
            lleg_len_pd.UpdateResult(lpendulum.dlen);
            Fl[0] = lleg_len_pd.result - GRAVITY_FF;
            rleg_len_pd.fdb = rpendulum.len;
            rleg_len_pd.UpdateResult(rpendulum.dlen);
            Fr[0] = rleg_len_pd.result - GRAVITY_FF;
            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);

            if (ins.accel[2] > 20.0f) 
            {
                pendulum_data.reset_len = true;
                chassis_state = NORMAL; 
            }
            break;

        case JUMP:

            switch (jump_stage) 
            {

            case DONT:
                chassis_state = NORMAL;
                break;

            case START:

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
                lqr.lqr_type = LQR_LOW;
                lqr.Update(lpendulum.len, rpendulum.len, false);

                Twl = lqr.Tout[0];
                Twr = lqr.Tout[1];
                Fl[1] = lqr.Tout[2];
                Fr[1] = lqr.Tout[3];
                lleg_len_pd.ref = NORMAL_LEG_LEN;
                rleg_len_pd.ref = NORMAL_LEG_LEN;
                lleg_len_pd.fdb = lpendulum.len;
                lleg_len_pd.UpdateResult(lpendulum.dlen);
                Fl[0] = lleg_len_pd.result - GRAVITY_FF;
                rleg_len_pd.fdb = rpendulum.len;
                rleg_len_pd.UpdateResult(rpendulum.dlen);
                Fr[0] = rleg_len_pd.result - GRAVITY_FF;


                if (cmd.ifjump)
                {
                    jump_stage = EXTENDING;
                }
                break;

            case EXTENDING:

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
                    jump_stage = INAIR;
                }
                break;

            case INAIR:

                refX[0] = observedX[0];
                refX[1] = observedX[1];
                refX[2] = observedX[2];
                refX[3] = observedX[3];
                refX[4] = 0.0f;
                refX[5] = observedX[5];
                refX[6] = 0.0f;
                refX[7] = observedX[7];
                refX[8] = observedX[8];
                refX[9] = observedX[9];
                lqr.lqr_type = LQR_LOW;
                lqr.Update(lpendulum.len, rpendulum.len, false);

                Twl = 0.0f;
                Twr = 0.0f;
                Fl[1] = lqr.Tout[2];
                Fr[1] = lqr.Tout[3];
                lleg_len_pd.ref = MIN_LEG_LEN;
                rleg_len_pd.ref = MIN_LEG_LEN;
                lleg_len_pd.fdb = lpendulum.len;
                lleg_len_pd.UpdateResult(lpendulum.dlen);
                Fl[0] = lleg_len_pd.result;
                rleg_len_pd.fdb = rpendulum.len;
                rleg_len_pd.UpdateResult(rpendulum.dlen);
                Fr[0] = rleg_len_pd.result;
                if (0.5f*(lpendulum.len+rpendulum.len)<=0.16f)
                {
                    jump_stage = LANDING;
                }
                break;

            case LANDING:

                odom.Reset();

                refX[0] = observedX[0];
                refX[1] = observedX[1];
                refX[2] = observedX[2];
                refX[3] = observedX[3];
                refX[4] = 0.0f;
                refX[5] = observedX[5];
                refX[6] = 0.0f;
                refX[7] = observedX[7];
                refX[8] = observedX[8];
                refX[9] = observedX[9];
                lqr.lqr_type = LQR_LOW;
                lqr.Update(lpendulum.len, rpendulum.len, false);

                Twl = 0.0f;
                Twr = 0.0f;
                Fl[1] = lqr.Tout[2];
                Fr[1] = lqr.Tout[3];
                lleg_len_pd.ref = NORMAL_LEG_LEN;
                rleg_len_pd.ref = NORMAL_LEG_LEN;
                lleg_len_pd.fdb = lpendulum.len;
                lleg_len_pd.UpdateResult(lpendulum.dlen);
                Fl[0] = lleg_len_pd.result;
                rleg_len_pd.fdb = rpendulum.len;
                rleg_len_pd.UpdateResult(rpendulum.dlen);
                Fr[0] = rleg_len_pd.result;
                if (lpendulum.N > 100.0f && rpendulum.N > 100.0f)
                {
                    odom.Reset();
                    jump_stage = DONT;
                    chassis_state = NORMAL;
                }
                break;
            }
            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);
            break;

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
        pendulum_debug.yaw_dot = dyaw;
        pendulum_debug.Twl = Twl;
        pendulum_debug.Twr = Twr;
        pendulum_debug.Tpl = lqr.Tout[2];
        pendulum_debug.Tpr = lqr.Tout[3];
        pendulum_debug.Fl = Fl[0];
        pendulum_debug.Fr = Fr[0];
        // pendulum_debug.Flreal = lpendulum.Freal;
        // pendulum_debug.Frreal = rpendulum.Freal;
        pendulum_debug.lneutral = lpendulum.neutral;
        pendulum_debug.rneutral = rpendulum.neutral;
        pendulum_debug.lflat = lpendulum.flat;
        pendulum_debug.rflat = rpendulum.flat;
        pendulum_debug.Nl = lpendulum.N;
        pendulum_debug.Nr = rpendulum.N;
        pendulum_debug.N = N;
        pendulum_debug.offground = (chassis_state == OFFGROUND);
        lleg_len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
        rleg_len_pd.Tuning(lenpd_tuning.kp, lenpd_tuning.ki, lenpd_tuning.kd);
        roll_pd.Tuning(rollpd_tuning.kp, rollpd_tuning.ki, rollpd_tuning.kd);
    #endif

        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));
    }
}
