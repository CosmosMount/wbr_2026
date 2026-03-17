#include "bsp_usart.hpp"
#include "XRobot.hpp"
#include "referee.hpp"
// #include "ServiceRemoter.hpp"

/*------------全局变量------------*/
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

#define XROBOT_IMUDATA_SIZE 59

__attribute__((section (".RAM_D1"))) uint8_t UART7RxBuffer[XROBOT_IMUDATA_SIZE] = {0};
__attribute__((section (".RAM_D1"))) uint8_t USART1RxBuffer[1] = {0};
__attribute__((section (".RAM_D1"))) uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM] = {0};

static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t);

/**
 * @brief  Configures the USART.
 * @param  None
 * @retval None
 */

void USART_Init()
{
  // uart5 双缓冲区初始化
  // USART_RxDMA_MultiBuffer_Init(&huart5, (uint32_t *)SBUS_MultiRx_Buf[0], (uint32_t *)SBUS_MultiRx_Buf[1], SBUS_RX_BUF_NUM);
  // uart7
  __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_uart7_rx, DMA_IT_TC);
  __HAL_DMA_DISABLE_IT(&hdma_uart7_tx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_uart7_tx, DMA_IT_TC);
  HAL_UART_Receive_DMA(&huart7, UART7RxBuffer, XROBOT_IMUDATA_SIZE);
  // HAL_UART_Receive_IT(&huart7, UART7RxBuffer, XROBOT_IMUDATA_SIZE);

  // usart1
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_TC);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_usart1_tx, DMA_IT_TC);
  HAL_UART_Receive_DMA(&huart1, USART1RxBuffer, 1);
  // HAL_UART_Receive_IT(&huart1, USART1RxBuffer, 1);
}

/**
 * @brief  Init the multi_buffer DMA Transfer with interrupt enabled.
 * @param  huart       pointer to a UART_HandleTypeDef structure that contains
 *                     the configuration information for the specified USART Stream.
 * @param  SrcAddress pointer to The source memory Buffer address
 * @param  DstAddress pointer to The destination memory Buffer address
 * @param  SecondMemAddress pointer to The second memory Buffer address in case of multi buffer Transfer
 * @param  DataLength The length of data to be transferred from source to destination
 * @retval none
 */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{

  huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

  huart->RxXferSize = DataLength * 2;

  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  do
  {
    __HAL_DMA_DISABLE(huart->hdmarx);
  } while (((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
  ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

  /* Configure the destination memory Buffer address */
  ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart7)
  {
    //清空cache
    DMA_Cache_PrepareForReceive(UART7RxBuffer, XROBOT_IMUDATA_SIZE);

    // XROBOT_IMU::Instance()->ProcessPacket(UART7RxBuffer, XROBOT_IMUDATA_SIZE);
    memcpy(&XROBOT_IMU::Instance()->xrobot_data.RxBuffer, UART7RxBuffer+1, 58);
    HAL_UART_Receive_DMA(&huart7, UART7RxBuffer, XROBOT_IMUDATA_SIZE);
  }
  else if (huart == &huart1)
  {
    //清空cache
    DMA_Cache_PrepareForReceive(USART1RxBuffer, 1);
    //
    // Referee::Instance()->Referee_Rx_Queue.Push(USART1RxBuffer[0]);
    HAL_UART_Receive_DMA(&huart1, USART1RxBuffer, 1);
  }
}
