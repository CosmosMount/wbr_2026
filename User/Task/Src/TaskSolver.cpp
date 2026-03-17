#include "GM6020.hpp"
#include "main.h"

#include "om.h"
#include "bsp_can.hpp"
#include "bsp_dwt.hpp"

#include "LK9025.hpp"
#include "LK8016.hpp"
#include "DJIMotorHandler.hpp"
#include "LKMotorHandler.hpp"

#include "math.hpp"
#include "filter.hpp"
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
uint8_t SolverThreadStack[4096] = {0};

LKMotorHandler *LKmotorhandler = LKMotorHandler::Instance();

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
};
__attribute__((section(".RAM_D3"))) solver_debug_t solver_debug;
#endif

#ifdef USE_MODEL_A
[[noreturn]] void SolverThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    LK9025 RWheel;
    LK9025 LWheel;

    LK8016 RJoint4;
    LK8016 RJoint1;

    LK8016 LJoint4;
    LK8016 LJoint1;

    LKMotorHandler::Instance()->registerMotor(&LJoint4, &hfdcan1, 0x141);
    LJoint4.currentSet = 0;
    LJoint4.offset = LJOINT4_OFFSET;
    LKMotorHandler::Instance()->registerMotor(&LJoint1, &hfdcan1, 0x142);
    LJoint1.currentSet = 0;
    LJoint1.offset = LJOINT1_OFFSET;
    LKMotorHandler::Instance()->registerMotor(&RJoint4, &hfdcan1, 0x143);
    RJoint4.currentSet = 0;
    RJoint4.offset = RJOINT4_OFFSET;
    LKMotorHandler::Instance()->registerMotor(&RJoint1, &hfdcan1, 0x144);
    RJoint1.currentSet = 0;
    RJoint1.offset = RJOINT1_OFFSET;
    LKMotorHandler::Instance()->registerMotor(&LWheel, &hfdcan3, 0x141);
    LWheel.currentSet = 0;
    LKMotorHandler::Instance()->registerMotor(&RWheel, &hfdcan3, 0x142);
    RWheel.currentSet = 0;

    constexpr float Tk_LK9025 = 195.3125f; // 2000 / (0.32f * 32.0f) 0.32：扭矩常数，32.0：电流实际最大值，2000.0：电流输入最大值
    constexpr float Tk_LK8016 = 43.4028f;  // 2000 / (0.24f * 32.0f * 6.0f) 0.24：扭矩常数，6：减速比，32.0：电流实际最大值，2000.0：电流数值范围

    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *pendulumctrl_suber = om_subscribe(om_find_topic("pendulumctrl", UINT32_MAX));
    msg_ctrl_t pendulumctrl{};
    om_suber_t  *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};

    om_topic_t *solverfdb_topic = om_config_topic(nullptr, "ca", "solverfdb", sizeof(msg_solver_t));
    msg_solver_t solverfdb{};
    om_topic_t *odom_pub = om_config_topic(nullptr, "ca", "odom", sizeof(msg_odometry_t));
    msg_odometry_t odom_data{};

    cVMCSolver Lsolver;
    cVMCSolver Rsolver;

    Odometry odom;

    float Lqdot[2] = {0};
    float Rqdot[2] = {0};

    float Lxdot[2] = {0};
    float Rxdot[2] = {0};

    float LTp[2] = {0};
    float RTp[2] = {0};

    float thread_start_time;

    for (;;)
    {
        thread_start_time = tx_time_get();
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(pendulumctrl_suber, &pendulumctrl, false);

        Lsolver.Resolve(-LJoint4.motorFeedback.positionFdb, PI-LJoint1.motorFeedback.positionFdb);
        Rsolver.Resolve(RJoint4.motorFeedback.positionFdb, PI+RJoint1.motorFeedback.positionFdb);

        solverfdb.llen = Lsolver.GetPendulumLen();
        solverfdb.rlen = Rsolver.GetPendulumLen();

        Lqdot[0] = -LJoint1.motorFeedback.speedFdb;
        Lqdot[1] = -LJoint4.motorFeedback.speedFdb;
        Rqdot[0] = RJoint1.motorFeedback.speedFdb;
        Rqdot[1] = RJoint4.motorFeedback.speedFdb;

        Lsolver.VMCVelCal(Lqdot, Lxdot);
        Rsolver.VMCVelCal(Rqdot, Rxdot);

        solverfdb.llen_dot = Lxdot[0];
        solverfdb.rlen_dot = Rxdot[0];
        solverfdb.lphi = Lsolver.GetPendulumRadian();
        solverfdb.rphi = Rsolver.GetPendulumRadian();
        solverfdb.lphi_dot = Lxdot[1];
        solverfdb.rphi_dot = Rxdot[1];

        float vel = 0.5f * (LWheel.motorFeedback.speedFdb - RWheel.motorFeedback.speedFdb) * WHEEL_RADIUS;
        odom_data = odom.Update(ins.quaternion, ins.accel, vel, ins.yaw);

        om_publish(solverfdb_topic, &solverfdb, sizeof(msg_solver_t), true, false);
        om_publish(odom_pub, &odom_data, sizeof(msg_odometry_t), true, false);

        Lsolver.VMCCal(pendulumctrl.Tl, LTp);
        Rsolver.VMCCal(pendulumctrl.Tr, RTp);

        LJoint4.currentSet = -Numeric::FloatConstrain(LTp[1], -MAX_HIP_TOR, MAX_HIP_TOR) * Tk_LK8016;
        LJoint1.currentSet = -Numeric::FloatConstrain(LTp[0], -MAX_HIP_TOR, MAX_HIP_TOR) * Tk_LK8016;
        RJoint4.currentSet = Numeric::FloatConstrain(RTp[1], -MAX_HIP_TOR, MAX_HIP_TOR) * Tk_LK8016;
        RJoint1.currentSet = Numeric::FloatConstrain(RTp[0], -MAX_HIP_TOR, MAX_HIP_TOR) * Tk_LK8016;

        LWheel.currentSet = Numeric::FloatConstrain(pendulumctrl.Twl, -MAX_WHEEL_TOR, MAX_WHEEL_TOR) * Tk_LK9025;
        RWheel.currentSet = -Numeric::FloatConstrain(pendulumctrl.Twr, -MAX_WHEEL_TOR, MAX_WHEEL_TOR) * Tk_LK9025;

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

        solver_debug.ljoint4_tor = LTp[0];
        solver_debug.ljoint1_tor = LTp[1];
        solver_debug.rjoint4_tor = RTp[0];
        solver_debug.rjoint1_tor = RTp[1];
        solver_debug.rwheel_tor_ref = -pendulumctrl.Twr;
        solver_debug.lwheel_tor_ref = pendulumctrl.Twl;
        solver_debug.rwheel_tor_fdb = RWheel.motorFeedback.torqueFdb;
        solver_debug.lwheel_tor_fdb = LWheel.motorFeedback.torqueFdb;

        solver_debug.ljoint4_pos = LJoint4.motorFeedback.positionFdb;
        solver_debug.ljoint1_pos = LJoint1.motorFeedback.positionFdb;
        solver_debug.rjoint4_pos = RJoint4.motorFeedback.positionFdb;
        solver_debug.rjoint1_pos = RJoint1.motorFeedback.positionFdb;

        solver_debug.lphi1dot = Lqdot[1];
        solver_debug.lphi4dot = Lqdot[0];
        solver_debug.rphi1dot = Rqdot[1];
        solver_debug.rphi4dot = Rqdot[0];
    #endif

        if (remoter.ctrl_sw == Relax || remoter.offline)
        {
            LJoint4.currentSet = 0;
            LJoint1.currentSet = 0;
            RJoint4.currentSet = 0;
            RJoint1.currentSet = 0;

            LWheel.currentSet = 0;
            RWheel.currentSet = 0;

            odom.Reset();
        }
        
        LKMotorHandler::Instance()->sendControlData();
        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));
    }
}
#endif
