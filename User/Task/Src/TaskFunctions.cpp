#include "DJIMotor.hpp"
#include "fast_math_functions.h"
#include "tx_api.h"
#include "vmc.hpp"
#include "om.h"
#include "usart.h"

#include "bsp_dwt.hpp"
#include "math.hpp"
#include "pid.hpp"
#include "lqr.hpp"
#include "slope.hpp"
#include "supercap.hpp"
#include "magicmsgs.hpp"

#include "config_comm.hpp"
#include "config_referee.hpp"
#include "config_chassis.hpp"
#include "config_remoter.hpp"

#include "M2006.hpp"
#include "M3508.hpp"
#include "GM6020.hpp"
#include "DJIMotorHandler.hpp"

using namespace chassis;

TX_THREAD FunctionThread;
uint8_t FunctionThreadStack[4096] = {0};
TX_SEMAPHORE TOFGot;
TX_SEMAPHORE FunctionThreadSem;

extern uint8_t CmdMsg[8];
__attribute__((section(".RAM_D1"))) uint8_t tof_rx[TOF_DATA_SIZE];

extern TX_SEMAPHORE IMUThreadSem;

#ifdef DEBUG
float debug_temp;
float debug_dist;
float debug_relativeangle;
bool debug_tof_valid;
__attribute__((section(".RAM_D3"))) msg_remoter_t debug_remoter;
comm_cmd_t *cmd_msg_debug;
typedef struct
{
    bool yaw_init;
    float yawmotor_spd;
    float yawmotor_cur;
    float yawmotor_pos;
    float tri_spd;
    float tri_cur;
} debug_motor_t;
debug_motor_t debug_motor;
SuperCap* debug_supercap = SuperCap::Instance();
#endif

[[noreturn]] void FunctionThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    /* Control Signal Initialization */
    float x_maintain;
    bool maintained_x = false;
    float yaw_maintain;
    bool maintained_yaw = false;
    bool pre_stair = false;

    /* Slope Updaters */
    SLOPE yaw_updater(0.0f, 0.006f);
    SLOPE vx_updater(0.0f,0.006f);
    SLOPE vy_updater(0.0f,0.006f);

    /* om publishers */
    om_topic_t *cmd_topic = om_config_topic(nullptr, "ca", "cmd", sizeof(msg_cmd_t));
    msg_cmd_t cmd{};
    om_topic_t *chassisui_topic = om_config_topic(nullptr, "ca", "chassisui", sizeof(msg_chassisui_t));
    msg_chassisui_t chassisui{};

    /* om subscribers */
    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};
    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *pendulum_suber = om_subscribe(om_find_topic("pendulum", UINT32_MAX));
    msg_pendulum_t pendulum_data{};
    om_suber_t *referee_suber = om_subscribe(om_find_topic("referee", UINT32_MAX));
    msg_referee_t referee_data{};

    SuperCap::Instance()->Init();

    /* Gimbal motors on chassis */
    GM6020 yaw_motor;
    yaw_motor.gearBox = GearBox_None;
    M2006 trigger_motor;
    trigger_motor.controlMode = DJIMotor::SPD_MODE;
    trigger_motor.gearBox = GearBox_None; // 使用速度×36，其实精度更高
    trigger_motor.speedPid.kp = 100.0f;
    bool yaw_init = false, lock_offset = false, comm_lost = false;
    float distance1, distance2, front_offset, relative_angle;
    int16_t prev_yaw_cur = 0;
    uint32_t comm_lost_cnt = 0;
    
    /* Communication with Gimbal */
    uint8_t CommMsg[8] = {0};
    comm_chassis_t chassis_msg{};

    DJIMotorHandler::Instance()->registerMotor(&yaw_motor, &hfdcan2, 0x205);
    DJIMotorHandler::Instance()->registerMotor(&trigger_motor, &hfdcan2, 0x203);

    for (;;)
    {

        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(pendulum_suber, &pendulum_data, false);
        om_suber_export(referee_suber, &referee_data, false);

        /* Receive and Check TOF Msg */
        bool tof_valid = false;
        tof_data_t *tof_raw =  reinterpret_cast<tof_data_t *>(tof_rx);
        if (tof_raw->header[0]==0x59 && tof_raw->header[1]==0x59)
        {
            uint8_t checksum = 0;
            for (int i = 0; i < 8; i++)
            {
                checksum += ((uint8_t*)tof_raw)[i];
            }
            if (checksum == tof_raw->check_sum)
            {
                tof_valid = true;
            }
        }
        float tof_distance = static_cast<float>(tof_raw->distance) * 1.0f; // cm
        float tof_temp = static_cast<float>(tof_raw->temp_raw) / 8.0f - 256.0f; // °C

        comm_cmd_t *cmd_msg = reinterpret_cast<comm_cmd_t*>(CmdMsg);
        cmd_msg_debug = cmd_msg;
        if (cmd_msg->yaw_cur == prev_yaw_cur)
        {
            comm_lost_cnt++;
            if (comm_lost_cnt > 100)
            {
                comm_lost = true;
            }
        }
        else
        {
            comm_lost_cnt = 0;
            comm_lost = false;
        }

    #ifdef GIMBAL_ONLY
        chassis_msg.inited = true;

        if (!cmd_msg->ifmove)
        {
            yaw_motor.currentSet = 0;
            trigger_motor.speedSet = 0.0f;
        }
        else 
        {
            yaw_motor.currentSet = cmd_msg->yaw_cur;
            trigger_motor.speedSet = cmd_msg->tri_spd*36.0f;
            trigger_motor.setOutput();
            DJIMotorHandler::Instance()->sendControlData();
        }
        
    #else

        /* Handle Gimbal Motors */
        distance1 = fabs(yaw_motor.motorFeedback.positionFdb-yaw_offset1);
        distance2 = fabs(yaw_motor.motorFeedback.positionFdb-yaw_offset2);
        if (!lock_offset)
        {
            front_offset = distance1 < distance2 ? yaw_offset1 : yaw_offset2;
        }

        relative_angle = LoopFloatConstrain(
            yaw_motor.motorFeedback.positionFdb - front_offset,
            -PI, PI
        );
        
        if (!cmd_msg->ifmove)
        {
            yaw_init = false;
            chassis_msg.inited = false;
            yaw_motor.currentSet = 0;
            trigger_motor.speedSet = 0.0f;
        }
        else if (!yaw_init)
        {
            yaw_motor.currentSet = (((yaw_motor.motorFeedback.positionFdb - yaw_offset1) > 0.0f) ? -1 : 1)*20000;
            if (fabs(yaw_motor.motorFeedback.positionFdb-yaw_offset1)<0.1f)
            {
                yaw_init = true;
                chassis_msg.inited = true;
            } 
        }
        else
        {
            yaw_motor.currentSet = cmd_msg->yaw_cur;
            trigger_motor.speedSet = cmd_msg->tri_spd*36.0f;
        }

        trigger_motor.speedSet = cmd_msg->tri_spd*36.0f;
        trigger_motor.setOutput();
        DJIMotorHandler::Instance()->sendControlData();

        cmd.roll = 0.0f;

        float len_target = Lmin;
        int8_t len_level = cmd_msg->len_level;
        if (len_level <= 0)
            len_target = Lmin;
        else if (len_level == 1)
            len_target = Lmid;
        else
            len_target = Lmax;

        if (isnan(cmd_msg->vx) || isnan(cmd_msg->vy) || comm_lost)
        {
            cmd.v = 0.0f;
            cmd.x = 0.0f;
            maintained_x = 0.0f; 
            cmd.len = Lmin;
            cmd.dlen = 0.0f;
            cmd.dyaw = 0.0f;
            cmd.move = false;
        }
        else if (!cmd_msg->ifmove)
        {
            cmd.v = 0.0f;
            cmd.x = 0.0f;
            maintained_x = 0.0f; 
            cmd.len = Lmin;
            cmd.dlen = 0.0f;
            cmd.dyaw = 0.0f;
            cmd.move = false;
        }
        else if (!pendulum_data.recovered)
        {
            cmd.v = 0.0f;
            cmd.x = 0.0f;
            maintained_x = 0.0f; 
            cmd.len = Lmin;
            cmd.dlen = 0.0f;
            cmd.dyaw = 0.0f;
            cmd.move = true;
        }
        else if (!yaw_init) 
        {
            cmd.v = 0.0f;
            cmd.x = 0.0f;
            maintained_x = 0.0f; 
            cmd.len = Lmin;
            cmd.dlen = 0.0f;
            cmd.dyaw = 0.0f;
            cmd.move = false;
        }
        else
        {
            cmd.move = true;
            cmd.len = len_target;
            cmd.dlen = 0.0f;

            cmd.spin = false;
            cmd.inair = false;
            cmd.iffly = false;
            cmd.ifjump = false;
            cmd.gostair = false;

            if (cmd_msg->ifspin)
            {
                lock_offset = true;
                cmd.spin = true;
                if (pendulum_data.len > 0.17f)
                {
                    cmd.dyaw = -relative_angle*4.0f;
                    cmd.v = 0.0f;
                }
                else
                {
                    cmd.dyaw = yaw_updater.UpdateVal(10.0f);
                    cmd.v = - vx_updater.UpdateVal(cmd_msg->vx*0.1f)*arm_sin_f32(relative_angle)
                            - vy_updater.UpdateVal(cmd_msg->vy*0.1f)*arm_cos_f32(relative_angle);
                }
                cmd.roll = 0.0f;
            }
            else 
            {
                if (cmd_msg->ifstair)
                {
                    cmd.gostair = true;
                }
                if (cmd_msg->iffly)
                {
                    cmd.iffly = true;
                }
                if (cmd_msg->ifjump)
                {
                    cmd.ifjump = true;
                }
                if (fabs(yaw_updater.GetVal()) > 1.0f)
                {
                    maintained_x = false;
                    cmd.x = pendulum_data.x;
                    cmd.dyaw = yaw_updater.UpdateVal(0.0f);
                }
                else
                {
                    if (fabs(cmd_msg->vx*0.1f) > 0.005f)
                    {
                        cmd.dyaw = -relative_angle*4.0f;
                        cmd.v = vx_updater.UpdateVal(cmd_msg->vx*0.1f*1.5f);
                    }
                    else if (fabs(cmd_msg->vy*0.1f) > 0.005f)
                    {
                        cmd.dyaw = -Numeric::LoopFloatConstrain(relative_angle + PI * 0.5f, -PI, PI) * 4.0f;
                        cmd.v = vy_updater.UpdateVal(cmd_msg->vy*0.1f*2.0f);
                    }
                    else
                    {
                        cmd.dyaw = -relative_angle*4.0f;
                        cmd.v = 0.0f;
                    }
                }
                cmd.roll = 0.0f;
            }
        }

        if (front_offset == yaw_offset2)
        {
            cmd.v *= -1.0f;
        }

        if ((fabsf(cmd.v) < 0.005f) && (fabsf(cmd.v-pendulum_data.v) < 0.5f))
        {
            if (!maintained_x)
            {
                x_maintain = pendulum_data.x;
                maintained_x = true;
            }
            cmd.x = x_maintain;
            if (!cmd.spin)
            {
                lock_offset = false;
            }
        }
        else
        {   
            maintained_x = false;
            cmd.x = pendulum_data.x+cmd.v*0.001f;
            lock_offset = true;
        }
        
        if (fabs(cmd.dyaw)<0.002f)
        {
            if (!maintained_yaw)
            {
                yaw_maintain = ins.total_yaw*DegreeToRad;
                maintained_yaw = true;
            }
            cmd.yaw = yaw_maintain;
        }
        else
        {
            maintained_yaw = false;
            if (cmd.spin)
                cmd.yaw = ins.total_yaw*DegreeToRad;
            else
                cmd.yaw = ins.total_yaw*DegreeToRad+cmd.dyaw*0.001f;
        }

        if (pendulum_data.len > 0.17f)
        {
            vx_updater.SetPath(0.006f-0.03f*(pendulum_data.len-0.17f));
            vy_updater.SetPath(0.006f-0.03f*(pendulum_data.len-0.17f));
        }
            
        else
        {
            vx_updater.SetPath(0.006f);
            vy_updater.SetPath(0.006f);
        }

        if (pendulum_data.len > 0.17f)
        {
            cmd.v *= 0.17f / pendulum_data.len;
        }
        if (!pendulum_data.normal)
        {
            cmd.v = 0.0f;
            vx_updater.SetDefault(0.0f);
            vy_updater.SetDefault(0.0f);
        }

    #endif

        chassis_msg.color = referee_data.robot_status.robot_id <= 9 ? 0 : 1;
        chassis_msg.level = referee_data.robot_status.robot_level;
        chassis_msg.heatlimit = referee_data.robot_status.shooter_barrel_heat_limit;
        chassis_msg.heatnow = referee_data.heat_now;

        SuperCap::Instance()->supercap_set.set.power_limit_set = referee_data.robot_status.chassis_power_limit;
        SuperCap::Instance()->SendCapData();

        memcpy(&CommMsg, reinterpret_cast<uint8_t*>(&chassis_msg), sizeof(comm_chassis_t));
        CAN_Transmit(&hfdcan3, 0xC1, CommMsg, 8);

        chassisui.relative_angle = yaw_motor.motorFeedback.positionFdb - yaw_offset2 + PI;
        chassisui.len = pendulum_data.len;
        chassisui.v = pendulum_data.v;

        /* Publish cmd msg */
        om_publish(cmd_topic, &cmd, sizeof(msg_cmd_t), true, false);
        om_publish(chassisui_topic, &chassisui, sizeof(msg_chassisui_t), true, false);
        pre_stair = cmd.gostair;

    #ifdef DEBUG
        debug_dist = tof_distance;
        debug_temp = tof_temp;
        debug_tof_valid = tof_valid;
        debug_remoter = remoter;
        // debug_relativeangle = relative_angle;

        debug_motor.yaw_init = yaw_init;
        debug_motor.yawmotor_pos = yaw_motor.motorFeedback.positionFdb;
        debug_motor.yawmotor_spd = yaw_motor.motorFeedback.speedFdb;
        debug_motor.yawmotor_cur = yaw_motor.motorFeedback.currentFdb;
        debug_motor.tri_spd = trigger_motor.motorFeedback.speedFdb;
        debug_motor.tri_cur = trigger_motor.motorFeedback.currentFdb;
    #endif
    
        /* Thread periodic delay */
        tx_semaphore_put(&FunctionThreadSem);
        tx_thread_sleep(1);
    }
}