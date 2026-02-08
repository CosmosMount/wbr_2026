#include "arm_math_types.h"
#include "main.h"
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
pid_tuning_t lenpd_tuning = {4000.0f, 0.0f, -2000.0f};
pid_tuning_t rollpd_tuning = {0.7f, 0.0f, 1.4f};
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
    // DMMotorHandler::Instance()->SaveZeroPosition(&RJoint1);
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
    PID rleg_len_pd(4000.0f, 0.0f, -2000.0f, 200.0f, 0.0f, PID_DVEL);
    PID lleg_len_pd(4000.0f, 0.0f, -2000.0f, 200.0f, 0.0f, PID_DVEL);
    /* roll */
    PID roll_pd(0.7f, 0.0f, 0.04f, 3.0f, 0.0f);
    /* state machine */
    chassis_state_e chassis_state = RELAX;
    jump_stage_e jump_stage = DONT;

    /* force&torque */
    float Twl = 0.0f;
    float Twr = 0.0f;
    float Fl[2] = {0.0f, 0.0f};
    float Fr[2] = {0.0f, 0.0f};

    float thread_start_time = 0.0f;

    for (;;)
    {
        thread_start_time = tx_time_get();
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

        if (!cmd.move)
            chassis_state = RELAX;

        switch (chassis_state) 
        {
        case RELAX:

            lpendulum.Relax();
            rpendulum.Relax();
            odom.Reset();
            pendulum_data.neutral = false;
            if (cmd.move)
                chassis_state = RECOVER;
            break;

        case RECOVER:

            chassis_state = FLATTEN;
            break;

        case FLATTEN:

            odom.Reset();
            if (lpendulum.flat) 
            {
                lpendulum.Relax();
            } 
            else    
            {
                lpendulum.DeltaPControl(-JOINT_FLAT_DELTA, 2.0f, 2.0f);
            }
            if (rpendulum.flat) 
            {
                rpendulum.Relax();
            } 
            else    
            {
                rpendulum.DeltaPControl(-JOINT_FLAT_DELTA, 2.0f, 2.0f);
            }
            if (lpendulum.flat && rpendulum.flat)   
            {
                chassis_state = NEUTRAL;
            }
            break;

        case GOSTAIR:
        
            odom.Reset();
            if (lpendulum.flat) 
            {
                lpendulum.Relax();
            }
            else    
            {
                lpendulum.DeltaPControl(JOINT_STAIR_DELTA, 2.0f, 1.0f);
            }
            if (rpendulum.flat) 
            {
                rpendulum.Relax();
            } 
            else    
            {
                rpendulum.DeltaPControl(JOINT_STAIR_DELTA, 2.0f, 1.0f);
            }
            if (lpendulum.flat && rpendulum.flat)   
            {
                chassis_state = NEUTRAL;
            }
            break;

        case NEUTRAL:

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
            lqr.Update(lpendulum.len, rpendulum.len);

            Twl = lqr.Tout[0];
            Twr = lqr.Tout[1];
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];

            lleg_len_pd.ref = 0.15f;
            rleg_len_pd.ref = 0.15f;

            lleg_len_pd.fdb = lpendulum.len;
            lleg_len_pd.UpdateResult(lpendulum.dlen);
            Fl[0] = lleg_len_pd.result;
            
            rleg_len_pd.fdb = rpendulum.len;
            rleg_len_pd.UpdateResult(rpendulum.dlen);
            Fr[0] = rleg_len_pd.result;

            if (lpendulum.len>0.18f) {Fl[1]=0.0f;}
            if (rpendulum.len>0.18f) {Fr[1]=0.0f;}
            if (Numeric::abs(lpendulum.alpha) > 0.8f) {Twl=0.0f;}  
            if (Numeric::abs(rpendulum.alpha) > 0.8f) {Twr=0.0f;}

            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);

            if (lpendulum.neutral && rpendulum.neutral)
            {
                pendulum_data.neutral = true;
                chassis_state = NORMAL;
                lpendulum.delta_init = false;
                rpendulum.delta_init = false;
            }
            break;

        case NORMAL:

            refX[0] = cmd.x;
            refX[1] = cmd.v;
            refX[2] = cmd.yaw;
            refX[3] = cmd.dyaw;
            refX[4] = 0.01f;
            refX[5] = 0.0f;
            refX[6] = 0.01f;
            refX[7] = 0.0f;
            refX[8] = 0.0f;
            refX[9] = 0.0f;

            // 用腿长长度判断LQR选择
            if ((lpendulum.len+rpendulum.len)*0.5f > 0.22f)
            {
                lqr.lqr_type = LQR_HIGH;
            }
            else{lqr.lqr_type = LQR_LOW;}
            lqr.Update(lpendulum.len, rpendulum.len);

            Twl = lqr.Tout[0];
            Twr = lqr.Tout[1];
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];

            roll_pd.ref = 0.0f;
            roll_pd.fdb = ins.roll*DegreeToRad;
            roll_pd.UpdateResult(ins.gyro_r);

            // if (cmd.spin)
            // {
                // bool rspin = ins.gyro_y < 0.0f;
                // float vout = rspin?fabs(vlwheel):fabs(vrwheel);
                // float vin = rspin?fabs(vrwheel):fabs(vlwheel);
                // float term1 = BODY_MASS * ((lpendulum.len+rpendulum.len) + WHEEL_DIST);
                // float term2 = (vout * vout - vin * vin);
                // float div = 8.0f * WHEEL_DIST * WHEEL_DIST;
                // float Fn = term1 * term2 / div;
                // rspin ? Fl[0] += Fn : Fr[0] += Fn;
                // roll_pd.result = 0.0f;
            // }

            lleg_len_pd.ref = cmd.len+roll_pd.result;//-0.03f
            rleg_len_pd.ref = cmd.len-roll_pd.result;//-0.03f
            lleg_len_pd.fdb = lpendulum.len;
            lleg_len_pd.UpdateResult(lpendulum.dlen);
            Fl[0] = lleg_len_pd.result;
            rleg_len_pd.fdb = rpendulum.len;
            rleg_len_pd.UpdateResult(rpendulum.dlen);
            Fr[0] = rleg_len_pd.result;

            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);
            
            if (lpendulum.N < 5.0f && rpendulum.N < 5.0f) { chassis_state = OFFGROUND; }
            if (cmd.gostair) { chassis_state = GOSTAIR; }
            if (cmd.prejump) { chassis_state = JUMP; jump_stage = START; }
            break;

        case OFFGROUND:

            odom.Reset();

            refX[0] = observedX[0];
            refX[1] = observedX[1];
            refX[2] = observedX[2];
            refX[3] = observedX[3];
            refX[4] = pitch;
            refX[5] = dpitch;
            refX[6] = pitch;
            refX[7] = dpitch;
            refX[8] = observedX[8];
            refX[9] = observedX[9];
            lqr.lqr_type = LQR_LOW;
            lqr.Update(lpendulum.len, rpendulum.len);

            Twl = -0.1f;
            Twr = -0.1f;
            Fl[1] = lqr.Tout[2];
            Fr[1] = lqr.Tout[3];
            lleg_len_pd.ref = 0.26f;
            rleg_len_pd.ref = 0.26f;
            lleg_len_pd.fdb = lpendulum.len;
            lleg_len_pd.UpdateResult(lpendulum.dlen);
            Fl[0] = lleg_len_pd.result;
            rleg_len_pd.fdb = rpendulum.len;
            rleg_len_pd.UpdateResult(rpendulum.dlen);
            Fr[0] = rleg_len_pd.result;
            lpendulum.TorqueControl(Fl, Twl);
            rpendulum.TorqueControl(Fr, Twr);

            if (lpendulum.N > 100.0f && rpendulum.N > 100.0f) { chassis_state = NORMAL; }
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
                lqr.Update(lpendulum.len, rpendulum.len);

                Twl = lqr.Tout[0];
                Twr = lqr.Tout[1];
                Fl[1] = lqr.Tout[2];
                Fr[1] = lqr.Tout[3];
                lleg_len_pd.ref = LEG_JUMP_START_LEN;
                rleg_len_pd.ref = LEG_JUMP_START_LEN;
                lleg_len_pd.fdb = lpendulum.len;
                lleg_len_pd.UpdateResult(lpendulum.dlen);
                Fl[0] = lleg_len_pd.result;
                rleg_len_pd.fdb = rpendulum.len;
                rleg_len_pd.UpdateResult(rpendulum.dlen);
                Fr[0] = rleg_len_pd.result;


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
                lqr.Update(lpendulum.len, rpendulum.len);

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
                refX[5] = 0.0f;
                refX[6] = 0.0f;
                refX[7] = 0.0f;
                refX[8] = observedX[8];
                refX[9] = observedX[9];
                lqr.lqr_type = LQR_LOW;
                lqr.Update(lpendulum.len, rpendulum.len);

                Twl = 1.0f;
                Twr = 1.0f;
                Fl[1] = lqr.Tout[2];//0.0f;
                Fr[1] = lqr.Tout[3];//0.0f;
                lleg_len_pd.ref = LEG_JUMP_AIR_LEN;
                rleg_len_pd.ref = LEG_JUMP_AIR_LEN;
                lleg_len_pd.fdb = lpendulum.len;
                lleg_len_pd.UpdateResult(lpendulum.dlen);
                Fl[0] = lleg_len_pd.result;
                rleg_len_pd.fdb = rpendulum.len;
                rleg_len_pd.UpdateResult(rpendulum.dlen);
                Fr[0] = rleg_len_pd.result;
                if (lpendulum.len<=0.15f && rpendulum.len<=0.15f)
                {
                    jump_stage = LANDING;
                }
                break;

            case LANDING:

                refX[0] = observedX[0];
                refX[1] = observedX[1];
                refX[2] = observedX[2];
                refX[3] = observedX[3];
                refX[4] = 0.0f;
                refX[5] = 0.0f;
                refX[6] = 0.0f;
                refX[7] = 0.0f;
                refX[8] = observedX[8];
                refX[9] = observedX[9];
                lqr.lqr_type = LQR_LOW;
                lqr.Update(lpendulum.len, rpendulum.len);

                Twl = 1.6f;
                Twr = 1.6f;
                Fl[1] = lqr.Tout[2];
                Fr[1] = lqr.Tout[3];
                lleg_len_pd.ref = 0.17f;
                rleg_len_pd.ref = 0.17f;
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
        
        pendulum_debug.x = odom.x;
        pendulum_debug.xref = cmd.x;
        pendulum_debug.v = odom.v;
        pendulum_debug.vref = cmd.v;
        pendulum_debug.l_ref = cmd.len;
        pendulum_debug.llen = lpendulum.len;
        pendulum_debug.rlen = rpendulum.len;
        pendulum_debug.alphal = lpendulum.alpha;
        pendulum_debug.alphar = rpendulum.alpha;
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
