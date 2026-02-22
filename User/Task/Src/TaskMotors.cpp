#include "fdcan.h"
#include "main.h"
#include "om_core.h"
#include "om_msg.h"
#include "tx_api.h"

#include "om.h"
#include "magicmsgs.hpp"

#include "pid.hpp"
#include "crc.hpp"

#include "M2006.hpp"
#include "M3508.hpp"
#include "GM6020.hpp"
#include "DJIMotorHandler.hpp"

TX_THREAD MotorsThread;
uint8_t MotorsThreadStack[2048] = {0};

[[noreturn]] void MotorsThreadFun(ULONG initial_input) 
{
    UNUSED(initial_input);

    GM6020 yaw_motor;
    M2006 trigger_motor;

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
        tx_thread_sleep(1);
    }
}