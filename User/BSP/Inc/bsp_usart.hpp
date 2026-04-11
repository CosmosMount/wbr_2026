#ifndef BSP_USART_HPP
#define BSP_USART_HPP

#include "main.h"
#include "string.h"
#include "stm32h7xx.h"
#include "usart.h"
#include "dma.h"
#include "adc.h"

enum USART_Mode
{
    USART_MODE_BLOCK = 0,
    USART_MODE_DMA = 1,
    USART_MODE_IT = 2
};

void USART_Init(void);
void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode);

#endif //  __BSP_USART_H