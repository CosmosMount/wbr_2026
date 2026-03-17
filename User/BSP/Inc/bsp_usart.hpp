#ifndef BSP_USART_HPP
#define BSP_USART_HPP

#include "main.h"
#include "string.h"
#include "stm32h7xx.h"
#include "usart.h"
#include "dma.h"
#include "bsp_cache.hpp"

#define SBUS_RX_BUF_NUM 18u


enum USART_Mode
{
    USART_MODE_BLOCK = 0,
    USART_MODE_DMA = 1,
    USART_MODE_IT = 2
  };

void USART_Init(void);

extern uint8_t SBUS_MultiRx_Buf[2][18u];



#endif //  __BSP_USART_H