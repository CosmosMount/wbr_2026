#include "DMMotorHandler.hpp"



// 电机数据转换因子

/**
 * @brief 构造函数，将所有值初始化
 */
DMMotorHandler::DMMotorHandler()
{
    for (uint8_t i = 0; i < MAX_DMMOTOR_NUM; i++)
    {

        DMMotorList[0][i] = nullptr;
        DMMotorList[1][i] = nullptr;
    }

    for (uint8_t i = 0; i < MAX_CAN_NUM; i++)
    {
        receive_num[i][0] = 0;
        receive_num[i][1] = 0;
    }
}

DMMotorHandler::~DMMotorHandler()
{
}

/**
 * @brief 阻塞地使能电机
 * @note 该函数不应该在中断中调用
 * @param motor 电机指针
 */
void DMMotorHandler::EnableMotor_Block(DMMotor *motor)
{
    uint32_t CAN_ID = motor->CAN_ID;
    switch (motor->ControlMode)
    {
    case DMMotor::RELAX_MODE:
        break;
    case DMMotor::MIT_MODE:
        break;
    case DMMotor::POS_SPD_MODE:
        CAN_ID += 0x100;
        break;
    case DMMotor::SPD_MODE:
        CAN_ID += 0x200;
        break;
    default:
        break;
    }

    uint16_t timeout = 0; // 超时计数，防止死循环。该函数不应该在高于1kHz的频率下调用
    do
    {
        timeout++;
        CAN_Transmit(&hfdcan1, CAN_ID, (uint8_t *)DMMotor::Enable_Frame, 8);
        if (timeout > 1000)
        {
            //printf("Enable DM Motor Timeout\n");
            break;
        }
    } while (motor->MotorState != DMMotor::MOTOR_OFFLINE && motor->MotorFeedback.ERR != DMMotor::ERR_ENABLE);
};

/**
 * @brief 使能电机
 * 发送一次使能帧
 */
void DMMotorHandler::EnableMotor(DMMotor *motor)
{
    uint32_t CAN_ID = motor->CAN_ID;
    switch (motor->ControlMode)
    {
    case DMMotor::RELAX_MODE:
        break;
    case DMMotor::MIT_MODE:
        break;
    case DMMotor::POS_SPD_MODE:
        CAN_ID += 0x100;
        break;
    case DMMotor::SPD_MODE:
        CAN_ID += 0x200;
        break;
    default:
        break;
    }

    CAN_Transmit(&hfdcan1, CAN_ID, (uint8_t *)DMMotor::Enable_Frame, 8);
};

/**
 * @brief 失能电机
 * 发送一次失能帧
 */
void DMMotorHandler::DisableMotor(DMMotor *motor)
{
    uint32_t CAN_ID = motor->CAN_ID;
    switch (motor->ControlMode)
    {
    case DMMotor::RELAX_MODE:
        break;
    case DMMotor::MIT_MODE:
        break;
    case DMMotor::POS_SPD_MODE:
        CAN_ID += 0x100;
        break;
    case DMMotor::SPD_MODE:
        CAN_ID += 0x200;
        break;
    default:
        break;
    }

    CAN_Transmit(&hfdcan1, CAN_ID, (uint8_t *)DMMotor::Disable_Frame, 8);
};

/**
 * @brief 注册电机
 * @param DMmotor 电机指针
 * @param CAN 电机所在的CAN口
 * @param canID 电机的ID
 */
void DMMotorHandler::RegisterMotor(DMMotor *DMmotor, FDCAN_HandleTypeDef *hcan, uint32_t canID)
{
    if (canID >= DM_CAN_ID + MAX_DMMOTOR_NUM)
    {
        canID = 0x04; // 默认CAN ID
    }
    DMmotor->CAN_ID = canID;
    DMmotor->hcan = hcan;

    if (canID >= 0x01 && canID <= 0x04)
    {
        if (DMmotor->hcan == &hfdcan1)
        {
            DMMotorList[0][canID - DM_CAN_ID] = DMmotor;
            if(canID >= 0x01 && canID <= 0x04){
                CAN1_0x3FE_Exist = true;
            }
            else if (canID >= 0x05 && canID <= 0x08){
                CAN1_0x4FE_Exist = true;
            }
            return;
        }

        else if (DMmotor->hcan == &hfdcan2)
        {
            DMMotorList[1][canID - DM_CAN_ID] = DMmotor;
            if(canID >= 0x01 && canID <= 0x04){
                CAN1_0x3FE_Exist = true;
            }
            else if (canID >= 0x05 && canID <= 0x08){
                CAN1_0x4FE_Exist = true;
            }
            return;
        }
    }
}

/**
 * @brief 遍历所有电机，发送控制数据
 * @note 需要在主循环中调用
 */
void DMMotorHandler::SendControlData()
{
    for (int i = 0; i < MAX_CAN_NUM; i++)
    {
        for (int j = 0; j < MAX_DMMOTOR_NUM; j++)
        {
            if (DMMotorList[i][j] != nullptr)
            {
                // SetOutput(DMMotorList[i][j]);
                // DMMotorList[i][j]->SetOutput();
                if(DMMotorList[i][j]->CAN_ID >= 0x01 && DMMotorList[i][j]->CAN_ID <= 0x04){
                    int index = (DMMotorList[i][j]->CAN_ID) * 2;
                    can1_send_data_0[index - 2] = DMMotorList[i][j]->currentSet >> 8;
                    can1_send_data_0[index - 1] = DMMotorList[i][j]->currentSet;
                }
                if(DMMotorList[i][j]->CAN_ID >= 0x05 && DMMotorList[i][j]->CAN_ID <= 0x08){
                    int index = (DMMotorList[i][j]->CAN_ID) * 2;
                    can2_send_data_0[index - 2] = DMMotorList[i][j]->currentSet >> 8;
                    can2_send_data_0[index - 1] = DMMotorList[i][j]->currentSet;
                }
            }
        }
    }
    if (CAN1_0x3FE_Exist)
        CAN_Transmit(&hfdcan1, 0x3FE, can1_send_data_0, 8); // 向CAN1发送数据，电机控制报文0x200
    if (CAN1_0x4FE_Exist)
        CAN_Transmit(&hfdcan1, 0x4FE, can1_send_data_1, 8); // 向CAN1发送数据，电机控制报文0x1FF
    if (CAN2_0x3FE_Exist)
        CAN_Transmit(&hfdcan2, 0x3FE, can2_send_data_0, 8); // 向CAN2发送数据，电机控制报文0x200
    if (CAN2_0x4FE_Exist)
        CAN_Transmit(&hfdcan2, 0x4FE, can2_send_data_1, 8); // 向CAN2发送数据，电机控制报文0x2FF
}


void DMMotorHandler::UpdateFeedback(FDCAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    if (hcan == &hfdcan1)
    {
            if(DMMotorList[0][index] != nullptr){
                DMMotorList[0][index]->ReceiveData(rx_data);
            }
    }
    else if (hcan == &hfdcan2)
    {
        
            if(DMMotorList[1][index] != nullptr){
                DMMotorList[1][index]->ReceiveData(rx_data);
            }
    }
}
    
/**
 * @brief 保存电机零点，发送一次保存零点帧
 * @param motor 电机指针
 */
void DMMotorHandler::SaveZeroPosition(DMMotor *motor)
{
    uint32_t CAN_ID = motor->CAN_ID;
    switch (motor->ControlMode)
    {
    case DMMotor::RELAX_MODE:
        break;
    case DMMotor::MIT_MODE:
        break;
    case DMMotor::POS_SPD_MODE:
        CAN_ID += 0x100;
        break;
    case DMMotor::SPD_MODE:
        CAN_ID += 0x200;
        break;
    default:
        break;
    }

    CAN_Transmit(&hfdcan1, CAN_ID, (uint8_t *)DMMotor::SaveZeroPosition_Frame, 8);
}

/**
 * @brief 清除电机错误，发送一次清除错误帧
 * @param motor 电机指针
 */
void DMMotorHandler::ClearError(DMMotor *motor)
{
    uint32_t CAN_ID = motor->CAN_ID;
    switch (motor->ControlMode)
    {
    case DMMotor::RELAX_MODE:
        break;
    case DMMotor::MIT_MODE:
        break;
    case DMMotor::POS_SPD_MODE:
        CAN_ID += 0x100;
        break;
    case DMMotor::SPD_MODE:
        CAN_ID += 0x200;
        break;
    default:
        break;
    }

    CAN_Transmit(&hfdcan1, CAN_ID, (uint8_t *)DMMotor::ClearError_Frame, 8);
}

/**
 * @brief 检查所有电机是否在线
 * @note 需要在主循环中调用
 */
void DMMotorHandler::AllMotorAliveCheck()
{
    for (uint8_t i = 0; i < MAX_CAN_NUM; i++)
    {
        for (int j = 0; j < MAX_DMMOTOR_NUM; j++)
        {
            if (DMMotorList[i][j] != nullptr)
            {
                DMMotorList[i][j]->AliveCheck();
            }
        }
    }
}
