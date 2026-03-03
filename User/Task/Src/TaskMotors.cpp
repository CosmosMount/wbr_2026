#include "fdcan.h"
#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"

#include "pid.hpp"
#include "crc.hpp"

#include "M2006.hpp"
#include "M3508.hpp"
#include "GM6020.hpp"
#include "DJIMotorHandler.hpp"

#include "config_chassis.hpp"

TX_THREAD MotorsThread;
uint8_t MotorsThreadStack[2048] = {0};

#ifdef DEBUG
typedef struct
{
    float yawmotor_spd;
    float yawmotor_cur;
    float tri_spd;
    float tri_cur;
} debug_motor_t;
debug_motor_t debug_motor;
#endif

[[noreturn]] void MotorsThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input);

    GM6020 yaw_motor;
    M2006 trigger_motor;
    trigger_motor.controlMode = DJIMotor::SPD_MODE;
    trigger_motor.gearBox = GearBox_M2006;
    trigger_motor.speedPid.kp = 100.0f;

    DJIMotorHandler::Instance()->registerMotor(&yaw_motor, &hfdcan2, 0x205);
    DJIMotorHandler::Instance()->registerMotor(&trigger_motor, &hfdcan2, 0x203);

    om_suber_t *cmd_suber = om_subscribe(om_find_topic("cmd", UINT32_MAX));
    msg_cmd_t cmd{};

    for (;;)
    {
        float thread_start_time = tx_time_get();
        om_suber_export(cmd_suber, &cmd, false);
        yaw_motor.currentSet = cmd.yawmotor_cur;
        trigger_motor.speedSet = cmd.tri_spd;
        trigger_motor.setOutput();
        DJIMotorHandler::Instance()->sendControlData();
    #ifdef DEBUG
        debug_motor.yawmotor_spd = yaw_motor.motorFeedback.speedFdb;
        debug_motor.yawmotor_cur = yaw_motor.motorFeedback.currentFdb;
        debug_motor.tri_spd = trigger_motor.motorFeedback.speedFdb;
        debug_motor.tri_cur = trigger_motor.motorFeedback.currentFdb;
    #endif
        tx_thread_sleep(1-thread_start_time);
    }
}