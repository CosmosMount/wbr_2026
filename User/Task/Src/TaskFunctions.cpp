#include "tx_api.h"
#include "vmc.hpp"
#include "om.h"
#include "usart.h"

#include "bsp_dwt.hpp"
#include "math.hpp"
#include "pid.hpp"
#include "lqr.hpp"
#include "slope.hpp"
#include "magicmsgs.hpp"

#include "config_chassis.hpp"
#include "config_remoter.hpp"
#include "config_comm.hpp"

TX_THREAD FunctionThread;
uint8_t FunctionThreadStack[2048] = {0};
TX_SEMAPHORE TOFGot;
TX_SEMAPHORE FunctionThreadSem;

extern uint8_t CmdMsg[16];
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
    float x_maintain;
    bool maintained_x = false;
    float yaw_maintain;
    bool maintained_yaw = false;
    bool not_jump = true;

    /* Slope Updaters */
    SLOPE yaw_updater(0.0f, 0.01f);
    SLOPE v_updater(0.0f,0.003f);
    SLOPE len_updater(0.13f,LEG_NORMAL_STEP);

    /* om publishers */
    om_topic_t *cmd_topic = om_config_topic(nullptr, "ca", "cmd", sizeof(msg_cmd_t));
    msg_cmd_t cmd{};

    /* om subscribers */
    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};
    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *pendulum_suber = om_subscribe(om_find_topic("pendulum", UINT32_MAX));
    msg_pendulum_t pendulum_data{};

    for (;;)
    {
        /* Thread start time */
        ULONG thread_start_time = tx_time_get();

        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(pendulum_suber, &pendulum_data, false);

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
            comm_cmd_t *cmd_msg = reinterpret_cast<comm_cmd_t*>(CmdMsg);

            cmd.roll = 0.0f;

            if (isnan(cmd_msg->dlen) || isnan(cmd_msg->v)) // 如果出现nan错误，将速度设定值设为0
            {
                cmd.v = 0.0f;
                cmd.len = 0.16f;
                cmd.dyaw = 0.0f;
                cmd.move = false;
            }
            else if (!cmd_msg->ifmove)
            {
                cmd.v = 0.0f;
                cmd.len = 0.16f;
                cmd.dyaw = 0.0f;
                cmd.move = false;
            }
            else
            {
                cmd.move = true;
                
                if (cmd_msg->ifspin)
                {
                    cmd.move = true;
                    cmd.roll = 0.0f;
                    cmd.dyaw = yaw_updater.UpdateVal(3.0f+5.0f*remoter.right_x);
                    cmd.v = 0.0f;
                    cmd.inair = false;
                    cmd.gostair = false;
                    cmd.spin = true;
                }
                else if (cmd_msg->ifjump)
                {
                    v_updater.SetPath(0.003f);
                    cmd.dyaw = yaw_updater.UpdateVal(-remoter.right_x*2.0f);//0.0f;//
                    cmd.v = v_updater.UpdateVal(remoter.left_y*1.5f);
                    // if ()
                    // {
                    //     cmd.prejump = true;
                    // }
                    // else if ()
                    // {
                    //     cmd.ifjump = true;
                    //     cmd.prejump = false;
                    // }
                    // else 
                    // {
                    //     cmd.prejump = false;
                    //     cmd.ifjump = false;
                    // }
                }
                else if (cmd_msg->ifstair)
                {
                    cmd.roll = 0.0f;
                    cmd.len += remoter.right_y*0.0008f;
                    cmd.len = FloatConstrain(cmd.len, MIN_LEG_LEN, MAX_LEG_LEN);
                    cmd.gostair = true;
                }
                else
                {
                    cmd.dyaw = yaw_updater.UpdateVal(-remoter.right_x*2.0f);
                    cmd.v = v_updater.UpdateVal(remoter.left_y*2.0f);
                    cmd.roll = 0.0f;
                    cmd.len += remoter.right_y*0.0008f;
                    cmd.len = FloatConstrain(cmd.len, MIN_LEG_LEN, MAX_LEG_LEN);
                    cmd.spin = false;
                    cmd.inair = false;
                    cmd.gostair = false;
                }
            }
            
        #else
            if (remoter.ctrl_sw == Relax || remoter.offline)
            {
                cmd.v = 0.0f;
                cmd.len = 0.16f;
                cmd.dyaw = 0.0f;
                cmd.move = false;
                cmd.inair = false;
                if (pendulum_data.len > 0.17f)
                    v_updater.SetPath(0.003f-0.0154f*(pendulum_data.len-0.17f));
                else
                    v_updater.SetPath(0.004f);
            }
            else if (remoter.ctrl_sw == Normal)
            {
                cmd.move = true;
                cmd.spin = false;
                if (remoter.jump_sw == None)
                {
                    cmd.dyaw = yaw_updater.UpdateVal(-remoter.right_x*2.0f);
                    cmd.v = v_updater.UpdateVal(remoter.left_y*2.0f);
                    cmd.roll = 0.0f;
                    cmd.len += remoter.right_y*0.0008f;
                    cmd.len = FloatConstrain(cmd.len, MIN_LEG_LEN, MAX_LEG_LEN);
                    cmd.inair = false;
                    cmd.gostair = false;
                }
                else
                {
                #ifdef JUMP_UP
                    v_updater.SetPath(0.003f);
                    cmd.dyaw = yaw_updater.UpdateVal(-remoter.right_x*2.0f);//0.0f;//
                    cmd.v = v_updater.UpdateVal(remoter.left_y*1.5f);
                    if (remoter.jump_sw == Prepared)
                    {
                        cmd.prejump = true;
                    }
                    else if (remoter.jump_sw == Jump)
                    {
                        cmd.ifjump = true;
                        cmd.prejump = false;
                    }
                    else 
                    {
                        cmd.prejump = false;
                        cmd.ifjump = false;
                    }
                #endif
                #ifdef STAIR_UP
                    if (remoter.jump_sw == Prepared)
                    {
                        
                        cmd.roll = 0.0f;
                        cmd.len += remoter.right_y*0.0008f;
                        cmd.len = FloatConstrain(cmd.len, MIN_LEG_LEN, MAX_LEG_LEN);
                        cmd.gostair = true;
                    }
                #endif
                }
            }
            else if (remoter.ctrl_sw == Spin)
            {
                cmd.move = true;
                cmd.roll = 0.0f;
                cmd.dyaw = yaw_updater.UpdateVal(3.0f+5.0f*remoter.right_x);
                cmd.v = 0.0f;
                cmd.inair = false;
                cmd.gostair = false;
                cmd.spin = true;
            }
        #endif
            if (!pendulum_data.neutral)
            {
                maintained_x = false;
                cmd.x = pendulum_data.x;
                cmd.v = pendulum_data.v;
            }
            else
            {
                if (fabsf(cmd.v) < 0.005f || remoter.ctrl_sw == Spin)
                {
                    if (!maintained_x)
                    {
                        x_maintain = pendulum_data.x;
                        maintained_x = true;
                    }
                    cmd.x = x_maintain;
                }
                else
                {   
                    maintained_x = false;
                    cmd.x = pendulum_data.x+cmd.v*0.001f;
                }
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
        tx_semaphore_put(&FunctionThreadSem);
        tx_thread_sleep(MIN(1, 1-(tx_time_get()-thread_start_time)));
    }
}