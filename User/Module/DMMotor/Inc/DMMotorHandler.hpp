#ifndef DMMOTORHANDLER_HPP
#define DMMOTORHANDLER_HPP

#include "main.h"

#include "bsp_can.hpp"

#include "DMMotor.hpp"

#include "Math.hpp"

#include "pid.hpp"

#define MAX_CAN_NUM 2     // 最大CAN口数量
#define MAX_DMMOTOR_NUM 4 // 每个CAN口最大电机数量

// 控制帧和反馈帧的ID偏移
#define DM_CAN_ID 0x01
#define DM_MASTER_ID 0x11

/**
 * @brief 电机控制类，管理所有的电机实例的数据和反馈
 */
class DMMotorHandler
{
public:
    /**
     *@brief 使用指针数组，存取所有电机的指针
     */
    DMMotor *DMMotorList[MAX_CAN_NUM][MAX_DMMOTOR_NUM];

    /**
     * @brief 每次接收到的数据个数，[i][j]代表第i个CAN口的第j个FIFO的数据个数
     */
    uint8_t receive_num[MAX_CAN_NUM][2];

    DMMotor::motor_measure_t can1_receive_data[8];
    DMMotor::motor_measure_t can2_receive_data[8];

    uint8_t can1_send_data_0[8]; // CAN1电机控制数据，用于控制0x201-0x204
    uint8_t can1_send_data_1[8]; // CAN1电机控制数据，用于控制0x205-0x208

    uint8_t can2_send_data_0[8]; // CAN2电机控制数据，用于控制0x201-0x204
    uint8_t can2_send_data_1[8]; // CAN2电机控制数据，用于控制0x205-0x208

    // 四个标志位，用于判断是否需要发送控制数据
    bool CAN1_0x3FE_Exist = false; // CAN1是否存在控制报文为0x200电机
    bool CAN1_0x4FE_Exist = false; // CAN1是否存在控制报文为0x1FF电机
    bool CAN2_0x3FE_Exist = false; // CAN2是否存在控制报文为0x200电机
    bool CAN2_0x4FE_Exist = false; // CAN2是否存在控制报文为0x1FF电机

    const float P_MIN = -12.5f; ///< 位置最小值
    const float P_MAX = 12.5f; ///< 位置最大值

    const float V_MIN = -45.0f; ///< 速度最小值
    const float V_MAX = 45.0f; ///< 速度最大值

    const float T_MIN = -54.0f; ///< 扭矩最小值
    const float T_MAX = 54.0f; ///< 扭矩最大值

    // CAN接收数据缓冲区
    // mcan_rx_message_t can0_rx1_buff[MAX_DMMOTOR_NUM]; // CAN0 RX FIFO1缓冲区
    // mcan_rx_message_t can0_rx0_buff[MAX_DMMOTOR_NUM]; // CAN0 RX FIFO0缓冲区

    // mcan_rx_message_t can1_rx1_buff[MAX_DMMOTOR_NUM]; // CAN1 RX FIFO1缓冲区
    // mcan_rx_message_t can1_rx0_buff[MAX_DMMOTOR_NUM]; // CAN1 RX FIFO0缓冲区

    // mcan_rx_message_t can2_rx1_buff[MAX_DMMOTOR_NUM]; // CAN2 RX FIFO1缓冲区
    // mcan_rx_message_t can2_rx0_buff[MAX_DMMOTOR_NUM]; // CAN2 RX FIFO0缓冲区

    // mcan_rx_message_t can3_rx1_buff[MAX_DMMOTOR_NUM]; // CAN3 RX FIFO1缓冲区
    // mcan_rx_message_t can3_rx0_buff[MAX_DMMOTOR_NUM]; // CAN3 RX FIFO0缓冲区

    // uint8_t Enable_Frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};           // 使能帧，DM电机需要初始化时发送该帧才能控制
    // uint8_t Disable_Frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};          // 失能帧
    // uint8_t SaveZeroPosition_Frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; // 保存零点帧
    // uint8_t ClearError_Frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};       // 清除错误帧

    DMMotorHandler();
    ~DMMotorHandler();

    void RegisterMotor(DMMotor *DMmotor, FDCAN_HandleTypeDef *hcan, uint32_t canID); // 使用指针作为参数

    void SendControlData();

    void UpdateFeedback(FDCAN_HandleTypeDef *hcan, uint8_t *rx_data, int index);// 更新电机反馈值

    //void Receive(DMMotor *motor, mcan_rx_message_t can_rx_buff);

    void EnableMotor_Block(DMMotor *motor);

    void EnableMotor(DMMotor *motor);

    void DisableMotor(DMMotor *motor);

    // void SetOutput(DMMotor *motor);

    void SaveZeroPosition(DMMotor *motor);

    void ClearError(DMMotor *motor);

    void AllMotorAliveCheck();

    static DMMotorHandler *Instance()
    {
        static DMMotorHandler instance;
        return &instance;
    }
};

#endif // GMMOTORHANDLER_HPP
