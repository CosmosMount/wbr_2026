#include "DJIMotor.hpp"
#include "DMMotor.hpp"
#include "GM6020.hpp"
#include "fast_math_functions.h"
#include "main.h"

#include "om.h"
#include "bsp_can.hpp"
#include "bsp_dwt.hpp"

#include "M3508.hpp"
#include "DM8009P.hpp"
#include "DMMotorHandler.hpp"
#include "DJIMotorHandler.hpp"

#include "math.hpp"
#include "odometry.hpp"
#include "om_core.h"
#include "om_msg.h"
#include "tx_api.h"
#include "utils.h"
#include "vmc.hpp"
#include "magicmsgs.hpp"

#include "config_chassis.hpp"

using namespace Numeric;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

TX_THREAD SolverThread;
TX_SEMAPHORE SolverThreadSem;
uint8_t SolverThreadStack[4096] = {0};


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
struct force_debug_t
{
    float Flreal;
    float Frreal;
    float Tleal;
    float Treal;
    float Nl;
    float Nr;
    float Pl;
    float Pr;
    float N;
    float Tljoint4;
    float Tljoint1;
    float Trjoint4;
    float Trjoint1;
};
__attribute__((section(".RAM_D3"))) solver_debug_t solver_debug;
__attribute__((section(".RAM_D3"))) force_debug_t force_debug;
#endif

[[noreturn]] void SolverThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

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
    DMMotorHandler::Instance()->EnableMotor(&LJoint4);

    DMMotorHandler::Instance()->registerMotor(&LJoint1, &hfdcan1, 0x02);
    LJoint1.controlMode = DMMotor::MIT_MODE;
    LJoint1.canType = DMMotor::DM_FDCAN;
    LJoint1.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor(&LJoint1);

    DMMotorHandler::Instance()->registerMotor(&RJoint4, &hfdcan1, 0x04);
    RJoint4.controlMode = DMMotor::MIT_MODE;
    RJoint4.canType = DMMotor::DM_FDCAN;
    RJoint4.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor(&RJoint4);

    DMMotorHandler::Instance()->registerMotor(&RJoint1, &hfdcan1, 0x03);
    RJoint1.controlMode = DMMotor::MIT_MODE;
    RJoint1.canType = DMMotor::DM_FDCAN;
    RJoint1.torqueSet = 0;
    DMMotorHandler::Instance()->EnableMotor(&RJoint1);
    
    constexpr float Tk_M3508 = 1400.0f;//2598.9848f; // 16384 / (0.02*286/17)Nm/A * 20A

    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *pendulumctrl_suber = om_subscribe(om_find_topic("pendulumctrl", UINT32_MAX));
    msg_ctrl_t pendulumctrl{};
    
    om_topic_t *solverfdb_topic = om_config_topic(nullptr, "ca", "solverfdb", sizeof(msg_solver_t));
    msg_solver_t solverfdb{};
    om_topic_t *odom_pub = om_config_topic(nullptr, "ca", "odom", sizeof(msg_odometry_t));
    msg_odometry_t odom_data{};
    om_suber_t *cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
    msg_cmd_t cmd{};

    cVMCSolver Lsolver;
    cVMCSolver Rsolver;

    Odometry odom;

    float Lqdot[2] = {0};
    float Rqdot[2] = {0};

    float Lxdot[2] = {0};
    float Rxdot[2] = {0};

    float LTp[2] = {0};
    float RTp[2] = {0};

    float LTpfdb[2] = {0};
    float RTpfdb[2] = {0};

    float TlRev[2] = {0};
    float TrRev[2] = {0};

    float prev_llen_dot = 0.0f;
    float prev_rlen_dot = 0.0f;

    uint32_t dm_msg_tick = 0;
    float thread_start_time = 0.0f;

    for (;;)
    {
        thread_start_time = tx_time_get();
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(cmd_suber, &cmd, false);
        om_suber_export(pendulumctrl_suber, &pendulumctrl, false);

        /* 将反馈值转化到模型对应角度，传入VMC */
        Lsolver.Resolve(PI+LJoint1.motorFeedback.positionFdb, LJoint4.motorFeedback.positionFdb);
        Rsolver.Resolve(PI-RJoint1.motorFeedback.positionFdb, -RJoint4.motorFeedback.positionFdb);

        /* VMC逆运动学解算 */
        solverfdb.llen = Lsolver.GetPendulumLen();
        solverfdb.rlen = Rsolver.GetPendulumLen();

        Lqdot[0] = LJoint1.motorFeedback.speedFdb;
        Lqdot[1] = LJoint4.motorFeedback.speedFdb;
        Rqdot[0] = -RJoint1.motorFeedback.speedFdb;
        Rqdot[1] = -RJoint4.motorFeedback.speedFdb;

        Lsolver.VMCVelCal(Lqdot, Lxdot);
        Rsolver.VMCVelCal(Rqdot, Rxdot);

        solverfdb.llen_dot = Lxdot[0];
        solverfdb.rlen_dot = Rxdot[0];
        solverfdb.lphi = Lsolver.GetPendulumRadian();
        solverfdb.rphi = Rsolver.GetPendulumRadian();
        solverfdb.lphi_dot = Lxdot[1];
        solverfdb.rphi_dot = Rxdot[1];

        float vel = 0.5f * (-LWheel.motorFeedback.speedFdb+RWheel.motorFeedback.speedFdb) * WHEEL_RADIUS;
        odom_data = odom.Update(ins.quaternion, ins.accel, vel, ins.yaw);

        solverfdb.lalpha = solverfdb.lphi-0.5f*Pi+ins.pitch*DegreeToRad;
        solverfdb.lalpha_dot = solverfdb.lphi_dot+ins.gyro_p;
        solverfdb.ralpha = solverfdb.rphi-0.5f*Pi+ins.pitch*DegreeToRad;
        solverfdb.ralpha_dot = solverfdb.rphi_dot+ins.gyro_p;

        /* VMC逆动力学解算 */
        LTpfdb[0] = LJoint1.motorFeedback.torqueFdb;
        LTpfdb[1] = LJoint4.motorFeedback.torqueFdb;
        RTpfdb[0] = -RJoint1.motorFeedback.torqueFdb;
        RTpfdb[1] = -RJoint4.motorFeedback.torqueFdb;

        Lsolver.VMCRevCal(TlRev, LTpfdb);
        Rsolver.VMCRevCal(TrRev, RTpfdb);

        float Pl = TlRev[0]*arm_cos_f32(solverfdb.lalpha)+TlRev[1]/solverfdb.llen*arm_sin_f32(solverfdb.lalpha);
        float Pr = TrRev[0]*arm_cos_f32(solverfdb.ralpha)+TrRev[1]/solverfdb.rlen*arm_sin_f32(solverfdb.ralpha);
        float ddlenl = solverfdb.llen_dot - prev_llen_dot;
        float ddlenr = solverfdb.rlen_dot - prev_rlen_dot;
        float Nl = Pl + WHEEL_MASS*(odom_data.a_z - ddlenl*arm_cos_f32(solverfdb.lalpha));
        float Nr = Pr + WHEEL_MASS*(odom_data.a_z - ddlenr*arm_cos_f32(solverfdb.ralpha));
        solverfdb.N = Nl + Nr;

        om_publish(solverfdb_topic, &solverfdb, sizeof(msg_solver_t), true, false);
        om_publish(odom_pub, &odom_data, sizeof(msg_odometry_t), true, false);

        prev_llen_dot = solverfdb.llen_dot;
        prev_rlen_dot = solverfdb.rlen_dot;

        Lsolver.VMCCal(pendulumctrl.Tl, LTp);
        Rsolver.VMCCal(pendulumctrl.Tr, RTp);

        LJoint1.torqueSet = Numeric::FloatConstrain(LTp[0], -MAX_HIP_TOR, MAX_HIP_TOR);
        LJoint4.torqueSet = Numeric::FloatConstrain(LTp[1], -MAX_HIP_TOR, MAX_HIP_TOR);
        RJoint1.torqueSet = -Numeric::FloatConstrain(RTp[0], -MAX_HIP_TOR, MAX_HIP_TOR);
        RJoint4.torqueSet = -Numeric::FloatConstrain(RTp[1], -MAX_HIP_TOR, MAX_HIP_TOR);

        LWheel.currentSet = -Numeric::FloatConstrain(pendulumctrl.Twl, -MAX_WHEEL_TOR, MAX_WHEEL_TOR) * Tk_M3508;
        RWheel.currentSet = Numeric::FloatConstrain(pendulumctrl.Twr, -MAX_WHEEL_TOR, MAX_WHEEL_TOR) * Tk_M3508;

    #ifdef DEBUG
        solver_debug.llength = solverfdb.llen;
        solver_debug.rlength = solverfdb.rlen;
        solver_debug.lphi = solverfdb.lphi;
        solver_debug.rphi = solverfdb.rphi;
        solver_debug.llength_dot = solverfdb.llen_dot;
        solver_debug.rlength_dot = solverfdb.rlen_dot;
        solver_debug.lphi_dot = solverfdb.lphi_dot;
        solver_debug.rphi_dot = solverfdb.rphi_dot;
        
        solver_debug.lphi1 = Lsolver.GetPhi1();
        solver_debug.lphi4 = Lsolver.GetPhi4();
        solver_debug.rphi1 = Rsolver.GetPhi1();
        solver_debug.rphi4 = Rsolver.GetPhi4();

        solver_debug.ljoint1_tor = LTp[0];
        solver_debug.ljoint4_tor = LTp[1];
        solver_debug.rjoint1_tor = -RTp[0];
        solver_debug.rjoint4_tor = -RTp[1];
        
        solver_debug.rwheel_tor_ref = pendulumctrl.Twr;
        solver_debug.lwheel_tor_ref = -pendulumctrl.Twl;
        solver_debug.rwheel_tor_fdb = RWheel.motorFeedback.currentFdb / Tk_M3508;
        solver_debug.lwheel_tor_fdb = LWheel.motorFeedback.currentFdb / Tk_M3508;

        solver_debug.ljoint4_pos = LJoint4.motorFeedback.positionFdb;
        solver_debug.ljoint1_pos = LJoint1.motorFeedback.positionFdb;
        solver_debug.rjoint4_pos = RJoint4.motorFeedback.positionFdb;
        solver_debug.rjoint1_pos = RJoint1.motorFeedback.positionFdb;

        solver_debug.lphi1dot = Lqdot[0];
        solver_debug.lphi4dot = Lqdot[1];
        solver_debug.rphi1dot = Rqdot[0];
        solver_debug.rphi4dot = Rqdot[1];

        solver_debug.lalpha = solverfdb.lalpha;
        solver_debug.ralpha = solverfdb.ralpha;

        // solver_debug = solverfdb;
        force_debug.Flreal = TlRev[0];
        force_debug.Frreal = TrRev[0];
        force_debug.Tleal = TlRev[1];
        force_debug.Treal = TrRev[1];
        force_debug.Pl = Pl;
        force_debug.Pr = Pr;
        force_debug.Nl = Nl;
        force_debug.Nr = Nr;
        force_debug.Tljoint4 = LJoint4.motorFeedback.torqueFdb;
        force_debug.Tljoint1 = LJoint1.motorFeedback.torqueFdb;
        force_debug.Trjoint4 = RJoint4.motorFeedback.torqueFdb;
        force_debug.Trjoint1 = RJoint1.motorFeedback.torqueFdb;
        force_debug.N = solverfdb.N;

        solver_debug.lwheel_pos = LWheel.motorFeedback.positionFdb;
        solver_debug.rwheel_pos = RWheel.motorFeedback.positionFdb;
    #endif

        if (!cmd.move)
        {
            LJoint4.torqueSet = 0;
            LJoint1.torqueSet = 0;
            RJoint4.torqueSet = 0;
            RJoint1.torqueSet = 0;

            LWheel.currentSet = 0;
            RWheel.currentSet = 0;

            odom.Reset();
        }

        // LJoint4.torqueSet = 0;
        // LJoint1.torqueSet = 0;
        // RJoint4.torqueSet = 0;
        // RJoint1.torqueSet = 0;

        // LWheel.currentSet = 0;
        // RWheel.currentSet = 0;


        LJoint1.SetOutput();
        LJoint4.SetOutput();
        RJoint1.SetOutput();
        RJoint4.SetOutput();


        dm_msg_tick ++;
        
        DJIMotorHandler::Instance()->sendControlData();
        tx_semaphore_put(&SolverThreadSem);
        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));
    }
}
