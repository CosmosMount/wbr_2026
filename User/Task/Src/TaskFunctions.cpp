#include "bsp_dwt.hpp"
#include "config_remoter.hpp"
#include "math.hpp"
#include "pid.hpp"
#include "lqr.hpp"
#include "slope.hpp"
#include "magicmsgs.hpp"
#include "config_chassis.hpp"
#include "tx_api.h"
#include "vmc.hpp"
#include "om.h"
#include <cstring>
#include "usart.h"

TX_THREAD FunctionThread;
uint8_t FunctionThreadStack[2048] = {0};
TX_SEMAPHORE TOFGot;

extern uint8_t xyAndRefAngleMsg[8];
extern uint8_t StateAnduiMsg[8];
__attribute__((section(".RAM_D1"))) uint8_t tof_rx[TOF_DATA_SIZE];

extern TX_SEMAPHORE IMUThreadSem;

#ifdef DEBUG
float debug_temp;
float debug_dist;
bool debug_tof_valid;
__attribute__((section(".RAM_D3"))) msg_remoter_t debug_remoter;
#endif

[[noreturn]] void FunctionThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    /* Control Signal Initialization */
#ifndef CHASSIS_ONLY
    chassis_mode_t mode = {NONE, NORMAL_ROTATE, DO_NOT_JUMP, NOT_FLY_MODE};
    uint16_t dlen_rx;
    uint16_t v_rx;
    float relativeangle;
#endif
    float x_maintain;
    bool maintained_x = false;

    /* Slope Updaters */
    SLOPE yaw_updater(0.0f, 0.01f);
    SLOPE v_updater(0.0f,0.005f);
    SLOPE len_updater(0.13f,0.0001f);

    /* om publishers */
    om_topic_t *cmd_topic = om_config_topic(nullptr, "ca", "cmd", sizeof(msg_cmd_t));
    msg_cmd_t cmd{};

    /* om subscribers */
    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};
    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *solver_suber = om_subscribe(om_find_topic("solverfdb", UINT32_MAX));
    msg_solver_t solver_fdb{};
    om_suber_t *odom_suber = om_subscribe(om_find_topic("odom", UINT32_MAX));
    msg_odometry_t odom{};

    for (;;)
    {
        /* Thread start time */
        ULONG thread_start_time = tx_time_get();

        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(solver_suber, &solver_fdb, false);
        om_suber_export(odom_suber, &odom, false);

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

        if (tx_semaphore_get(&IMUThreadSem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
        #ifndef CHASSIS_ONLY

            /* Receive Gimbal Msg */
            uint8_t state_msg = StateAnduiMsg[0];
            mode.chassis_mode = static_cast<chassis_mode_e>(state_msg & 0x03);
            mode.rotate_type = static_cast<rotate_ctrl_e>((state_msg >> 2) & 0x01);
            mode.jump_ctrl = static_cast<jump_ctrl_e>((state_msg >> 3) & 0x03);
            mode.fly_ctrl = static_cast<fly_ctrl_e>((state_msg >> 5) & 0x01);

            memcpy(&dlen_rx, xyAndRefAngleMsg, 2);                  // 将接收到的数据拷贝到Vx
            memcpy(&v_rx, xyAndRefAngleMsg + 2, 2);                 // 将接收到的数据拷贝到Vy
            memcpy(&relativeangle, xyAndRefAngleMsg + 4, 4);        // 将接收到的数据拷贝到RelativeAngle

            cmd.roll = 0.0f;

            if (isnan(dlen_rx) || isnan(v_rx) || isnan(relativeangle) || (mode.chassis_mode > 3) || (mode.rotate_type > 1) || (mode.jump_ctrl > 2)) // 如果出现nan错误，将速度设定值设为0
            {
                cmd.v = 0.0f;
                cmd.w = 0.0f;
                cmd.dlen = 0.0f;
                cmd.dyaw = 0.0f;
                relativeangle = 0.0f;
                cmd.move = false;
                mode.chassis_mode = NONE;
                mode.rotate_type = NORMAL_ROTATE;
                mode.jump_ctrl = DO_NOT_JUMP;
            }
            else if (mode.chassis_mode == NONE)
            {
                cmd.v = 0.0f;
                cmd.w = 0.0f;
                cmd.dlen = 0.0f;
                cmd.dyaw = 0.0f;
                cmd.move = false;
            }
            else if (mode.chassis_mode == NORMAL_MOVING_MODE)
            {
                cmd.move = true;
                if (fabsf(cmd.dlen) < 0.0005f)
                    cmd.dlen = 0.0f;
                if (fabsf(cmd.v) < 0.0005f)
                    cmd.v = 0.0f;
                if (fabsf(relativeangle) < 0.0001f)
                    relativeangle = 0.0f;

                /* v, dlen [0,60000] -> [-2,2] */
                cmd.dlen = len_updater.UpdateVal(((float)dlen_rx) / 15000.0f - 2.0f);
                cmd.v = v_updater.UpdateVal(((float)v_rx) / 15000.0f - 2.0f);
                
                if (mode.rotate_type == SPIN_ROTATE)
                {
                    cmd.w = 3.0f;
                    cmd.dyaw = 0.0f;
                }
                else 
                {
                    cmd.dyaw = yaw_updater.UpdateVal(relativeangle)*2.0f;
                    cmd.w = 0.0f;
                }

                if (fabsf(cmd.v) < 0.002f || mode.rotate_type == SPIN_ROTATE)
                {
                    if (!maintained_x)
                    {
                        x_maintain = odom.x;
                        maintained_x = true;
                    }
                    cmd.x = x_maintain;
                }
                else
                {   
                    maintained_x = false;
                    cmd.x = odom.x+cmd.v*0.001f;
                }
            }
            
        #else
            if (remoter.ctrl_sw == Relax || remoter.offline)
            {
                cmd.v = 0.0f;
                cmd.w = 0.0f;
                cmd.dlen = 0.0f;
                cmd.dyaw = 0.0f;
                cmd.move = false;
            }
            else if (remoter.ctrl_sw == Normal)
            {
                cmd.move = true;
                cmd.dyaw = yaw_updater.UpdateVal(-remoter.right_x);
                cmd.v = v_updater.UpdateVal(remoter.left_y*2.0f);
                cmd.roll = 0.0f;//remoter.right_x*0.1f;
                cmd.w = 0.0f;
            }
            else if (remoter.ctrl_sw == Spin)
            {
                cmd.move = true;
                cmd.w = 3.0f;
                cmd.dyaw = 0.0f;
                cmd.v = 0.0f;
            }

            if (fabsf(cmd.v) < 0.002f || remoter.ctrl_sw == Spin)
            {
                if (!maintained_x)
                {
                    x_maintain = odom.x;
                    maintained_x = true;
                }
                cmd.x = x_maintain;
            }
            else
            {   
                maintained_x = false;
                cmd.x = odom.x+cmd.v*0.001f;
            }
        #endif           
        }

        /* Publish cmd msg */
        om_publish(cmd_topic, &cmd, sizeof(msg_cmd_t), true, false);

    #ifdef DEBUG
        debug_dist = tof_distance;
        debug_temp = tof_temp;
        debug_tof_valid = tof_valid;
        debug_remoter = remoter;
    #endif
        /* Thread periodic delay */
        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));
    }
}