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
struct solver_debug_t
{
    float llength;
    float rlength;
    float lphi;
    float rphi;
    float llength_dot;
    float rlength_dot;
    float lphi_dot;
    float rphi_dot;
    float lphi1;
    float lphi4;
    float lphi1dot;
    float lphi4dot;
    float rphi1;
    float rphi4;
    float rphi1dot;
    float rphi4dot;
    float ljoint4_tor;
    float ljoint1_tor;
    float rjoint4_tor;
    float rjoint1_tor;
    float rwheel_tor_ref;
    float lwheel_tor_ref;
    float rwheel_tor_fdb;
    float lwheel_tor_fdb;
    float ljoint4_pos;
    float ljoint1_pos;
    float rjoint4_pos;
    float rjoint1_pos;
    float lwheel_pos;
    float rwheel_pos;
    float lalpha;
    float ralpha;
};
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
    float Nl;
    float Nr;
    float llendot;
    float rlendot;
    float yaw;
    float yawref;
    float yaw_dot;
    float yaw_out;
    bool lneutral;
    bool rneutral;
};
// struct force_debug_t
// {
//     float Flreal;
//     float Frreal;
//     float Tleal;
//     float Treal;
//     float Nl;
//     float Nr;
//     float Pl;
//     float Pr;
//     float N;
//     float Tljoint4;
//     float Tljoint1;
//     float Trjoint4;
//     float Trjoint1;
// };
msg_ins_t debug_ins;
pendulum_debug_t pendulum_debug;
__attribute__((section(".RAM_D3"))) solver_debug_t solver_debug;
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

    DJIMotorHandler::Instance()->registerMotor(&LWheel, &hfdcan2, 0x203);
    LWheel.currentSet = 0;
    LWheel.gearBox = GearBox_XRoll;
    DJIMotorHandler::Instance()->registerMotor(&RWheel, &hfdcan2, 0x204);
    RWheel.currentSet = 0;
    RWheel.gearBox = GearBox_XRoll;

    DMMotorHandler::Instance()->registerMotor(&LJoint4, &hfdcan1, 0x01);
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

    DMMotorHandler::Instance()->registerMotor(&RJoint1, &hfdcan1, 0x03);
    RJoint1.controlMode = DMMotor::MIT_MODE;
    RJoint1.canType = DMMotor::DM_FDCAN;
    RJoint1.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor_Block(&RJoint1);

    /* If need to save zero points */
    // DMMotorHandler::Instance()->SaveZeroPosition(&LJoint4);
    // DMMotorHandler::Instance()->SaveZeroPosition(&LJoint1);
    // DMMotorHandler::Instance()->SaveZeroPosition(&RJoint4);
    // DMMotorHandler::Instance()->SaveZeroPosition(&RJoint1);
    
    float ljoint1_flat_init = 0.0f;
    float ljoint4_flat_init = 0.0f;
    float rjoint1_flat_init = 0.0f;
    float rjoint4_flat_init = 0.0f;

    /* one message initialization */
    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
    msg_cmd_t cmd{};
    
    om_topic_t *pendulum_pub = om_config_topic(nullptr, "ca", "pendulum", sizeof(msg_pendulum_t));
    msg_pendulum_t pendulum_data{};

    /* solvers */
    PendulumSolver lsolver(false);
    PendulumSolver rsolver(true);
    /* odom */
    Odometry odom;
    /* lqr */
    LQR lqr;
    float Tout[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float observedX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float refX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    lqr.InitMatX(&refX[0], &observedX[0]);
    /* len */
    PID rleg_len_pd(2000.0f, 0.0f, -1000.0f, 200.0f, 0.0f, PID_DVEL);
    PID lleg_len_pd(2000.0f, 0.0f, -1000.0f, 200.0f, 0.0f, PID_DVEL);
    /* roll */
    PID roll_pd(0.7f, 0.0f, 0.01f, 3.0f, 0.0f);
    /* state machine */
    chassis_state_e chassis_state = RELAX;

    /* force&torque */
    float Twl = 0.0f;
    float Twr = 0.0f;
    constexpr float Tk_M3508 = 1400.0f;
    float Fl[2] = {0.0f, 0.0f};
    float Fr[2] = {0.0f, 0.0f};
    float Tl[2] = {0.0f, 0.0f};
    float Tr[2] = {0.0f, 0.0f};

    /* helper functions */
    auto lrelax = [&]()
    {
        LJoint4.torqueSet = 0; LJoint1.torqueSet = 0;
        LJoint1.speedSet = 0; LJoint4.speedSet = 0;
        LJoint1.positionSet = LJoint1.motorFeedback.positionFdb;
        LJoint4.positionSet = LJoint4.motorFeedback.positionFdb;

        LJoint1.KP = 0.0f; LJoint4.KP = 0.0f;
        LJoint1.KD = 0.0f; LJoint4.KD = 0.0f;

        LWheel.currentSet = 0;
    };

    auto rrelax = [&]()
    {
        RJoint4.torqueSet = 0; RJoint1.torqueSet = 0;
        RJoint1.speedSet = 0; RJoint4.speedSet = 0;
        RJoint1.positionSet = RJoint1.motorFeedback.positionFdb;
        RJoint4.positionSet = RJoint4.motorFeedback.positionFdb;

        RJoint1.KP = 0.0f; RJoint4.KP = 0.0f;
        RJoint1.KD = 0.0f; RJoint4.KD = 0.0f;

        RWheel.currentSet = 0;
    };

    float thread_start_time = 0.0f;
    bool flatten_initialized = false;

    for (;;)
    {
        thread_start_time = tx_time_get();
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(cmd_suber, &cmd, false);

        float pitch = ins.pitch*DegreeToRad;
        float dpitch = ins.gyro_p;
        float yaw = ins.total_yaw*DegreeToRad;
        float dyaw = ins.gyro_y;

        lsolver.Update(LJoint1.motorFeedback.positionFdb, LJoint4.motorFeedback.positionFdb, pitch, 
                      LJoint1.motorFeedback.speedFdb, LJoint4.motorFeedback.speedFdb, dpitch, 
                      LJoint1.motorFeedback.torqueFdb, LJoint4.motorFeedback.torqueFdb, odom.az);
        rsolver.Update(RJoint1.motorFeedback.positionFdb, RJoint4.motorFeedback.positionFdb, pitch, 
                      RJoint1.motorFeedback.speedFdb, RJoint4.motorFeedback.speedFdb, dpitch, 
                      RJoint1.motorFeedback.torqueFdb, RJoint4.motorFeedback.torqueFdb, odom.az);
        odom.Update(ins.quaternion, ins.accel, 
                    (-LWheel.motorFeedback.speedFdb+RWheel.motorFeedback.speedFdb)*WHEEL_RADIUS*0.5f, yaw);

        if (!cmd.move)
            chassis_state = RELAX;

        if (chassis_state == RELAX)
        {
            lrelax();
            rrelax();
            odom.Reset();
            flatten_initialized = false;
            pendulum_data.neutral = false;
            if (cmd.move)
                chassis_state = RECOVER;
        }
        else if (chassis_state == RECOVER)
        {
            chassis_state = FLATTEN;
        }
        else if (chassis_state == FLATTEN)
        {
            LJoint4.torqueSet = 0; LJoint1.torqueSet = 0; 
            RJoint4.torqueSet = 0; RJoint1.torqueSet = 0;
            if (!flatten_initialized)
            {
                ljoint1_flat_init = LJoint1.motorFeedback.positionFdb;
                ljoint4_flat_init = LJoint4.motorFeedback.positionFdb;
                rjoint1_flat_init = RJoint1.motorFeedback.positionFdb;
                rjoint4_flat_init = RJoint4.motorFeedback.positionFdb;
                flatten_initialized = true;
            }
            if (lsolver.flat)
                lrelax();
            else
            {
                LJoint1.positionSet = ljoint1_flat_init-JOINT_FLAT_DELTA;
                LJoint4.positionSet = ljoint4_flat_init-JOINT_FLAT_DELTA;
                LJoint1.speedSet = -0.1f; LJoint4.speedSet = -0.1f;
                LJoint1.KP = 1.0f; LJoint4.KP = 1.0f;
                LJoint1.KD = 1.0f; LJoint4.KD = 1.0f;
            }

            if (rsolver.flat)
                rrelax();
            else
            {
                RJoint1.positionSet = rjoint1_flat_init+JOINT_FLAT_DELTA;
                RJoint4.positionSet = rjoint4_flat_init+JOINT_FLAT_DELTA;
                RJoint1.speedSet = 0.1f; RJoint4.speedSet = 0.1f;
                RJoint1.KP = 1.0f; RJoint4.KP = 1.0f;
                RJoint1.KD = 1.0f; RJoint4.KD = 1.0f;
            }

            if (lsolver.flat && rsolver.flat)
            {
                chassis_state = NEUTRAL;
            }
        }
        else
        {
            observedX[0] = odom.x;
            observedX[1] = odom.v;
            observedX[2] = yaw;
            observedX[3] = dyaw;
            observedX[4] = lsolver.alpha;
            observedX[5] = lsolver.dalpha;
            observedX[6] = rsolver.alpha;
            observedX[7] = rsolver.dalpha;
            observedX[8] = pitch;
            observedX[9] = dpitch;

            refX[0] = cmd.x;//odom.x;//
            refX[1] = cmd.v;//odom.v;//
            refX[2] = cmd.yaw;
            refX[3] = cmd.w+cmd.dyaw;
            refX[4] = 0.06f;
            refX[5] = 0.0f;
            refX[6] = 0.06f;
            refX[7] = 0.0f;
            refX[8] = 0.0f;
            refX[9] = 0.0f;
            lqr.refreshLQRK(lsolver.len, rsolver.len);
            lqr.LQRCal(Tout);
            
            Twl = Tout[0];
            Twr = Tout[1];
            Fl[1] = Tout[2];
            Fr[1] = Tout[3];

            if (chassis_state == NEUTRAL)
            {
                lleg_len_pd.ref = 0.21f;
                rleg_len_pd.ref = 0.21f;
                if (lsolver.neutral && rsolver.neutral)
                {
                    pendulum_data.neutral = true;
                    chassis_state = NORMAL;
                }
            }
            else if (chassis_state == NORMAL)
            {
                float N = (lsolver.N+rsolver.N)*0.5f;
                roll_pd.ref = 0.0f;
                roll_pd.fdb = ins.roll*DegreeToRad;
                roll_pd.UpdateResult(ins.gyro_r);
                lleg_len_pd.ref = cmd.len+0.03f+roll_pd.result;
                rleg_len_pd.ref = cmd.len+0.03f-roll_pd.result;
                if (cmd.gostair)
                {
                    Fl[1] = 0.0f;
                    Fr[1] = 0.0f;
                }
                if (lsolver.neutral && rsolver.neutral && (N<20.0f || cmd.inair))
                {
                    Twl = 0.0f;
                    Twr = 0.0f;
                }
            }

            lleg_len_pd.fdb = lsolver.len;
            lleg_len_pd.UpdateResult(lsolver.dlen);
            Fl[0] = lleg_len_pd.result;
            
            rleg_len_pd.fdb = rsolver.len;
            rleg_len_pd.UpdateResult(rsolver.dlen);
            Fr[0] = rleg_len_pd.result;

            lsolver.ForwardDynamics(Fl, Tl);
            rsolver.ForwardDynamics(Fr, Tr);

            LJoint1.torqueSet = Numeric::FloatConstrain(Tl[0], -MAX_HIP_TOR, MAX_HIP_TOR);
            LJoint4.torqueSet = Numeric::FloatConstrain(Tl[1], -MAX_HIP_TOR, MAX_HIP_TOR);
            RJoint1.torqueSet = -Numeric::FloatConstrain(Tr[0], -MAX_HIP_TOR, MAX_HIP_TOR);
            RJoint4.torqueSet = -Numeric::FloatConstrain(Tr[1], -MAX_HIP_TOR, MAX_HIP_TOR);
            LWheel.currentSet = -Numeric::FloatConstrain(Twl, -MAX_WHEEL_TOR, MAX_WHEEL_TOR) * Tk_M3508;
            RWheel.currentSet = Numeric::FloatConstrain(Twr, -MAX_WHEEL_TOR, MAX_WHEEL_TOR) * Tk_M3508;
        }

        DMMotorHandler::Instance()->sendControlData();
        DJIMotorHandler::Instance()->sendControlData();

        pendulum_data.x = odom.x;
        pendulum_data.v = odom.v;
        pendulum_data.N = (lsolver.N+rsolver.N)*0.5f;
        om_publish(pendulum_pub, &pendulum_data, sizeof(msg_pendulum_t), true, false);  
        tx_semaphore_put(&PendulumThreadSem);

    #ifdef DEBUG
        debug_ins = ins;

        solver_debug.llength = lsolver.len;
        solver_debug.rlength = rsolver.len;
        solver_debug.lalpha = lsolver.alpha;
        solver_debug.ralpha = rsolver.alpha;
        solver_debug.ljoint1_pos = LJoint1.motorFeedback.positionFdb;
        solver_debug.ljoint4_pos = LJoint4.motorFeedback.positionFdb;
        solver_debug.rjoint1_pos = RJoint1.motorFeedback.positionFdb;
        solver_debug.rjoint4_pos = RJoint4.motorFeedback.positionFdb;
        solver_debug.lwheel_pos = LWheel.motorFeedback.positionFdb;
        solver_debug.rwheel_pos = RWheel.motorFeedback.positionFdb;
        
        pendulum_debug.x = odom.x;
        pendulum_debug.xref = cmd.x;
        pendulum_debug.v = odom.v;
        pendulum_debug.vref = cmd.v;
        pendulum_debug.llen = lsolver.len;
        pendulum_debug.rlen = rsolver.len;
        pendulum_debug.alphal = lsolver.alpha;
        pendulum_debug.alphar = rsolver.alpha;
        pendulum_debug.alphal_dot = lsolver.dalpha;
        pendulum_debug.alphar_dot = rsolver.dalpha;
        pendulum_debug.pitch = pitch;
        pendulum_debug.pitch_dot = dpitch;
        pendulum_debug.yaw = yaw;
        pendulum_debug.yawref = cmd.yaw;
        pendulum_debug.yaw_dot = dyaw;
        pendulum_debug.Twl = Twl;
        pendulum_debug.Twr = Twr;
        pendulum_debug.Tpl = Tout[2];
        pendulum_debug.Tpr = Tout[3];
        pendulum_debug.Fl = Fl[0];
        pendulum_debug.Fr = Fr[0];
        pendulum_debug.lneutral = lsolver.neutral;
        pendulum_debug.rneutral = rsolver.neutral;
        pendulum_debug.Nl = lsolver.N;
        pendulum_debug.Nr = rsolver.N;
    #endif

        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));
    }
}
