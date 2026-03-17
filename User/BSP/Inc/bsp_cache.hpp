#ifndef BSP_CACHE_HPP
#define BSP_CACHE_HPP

#include "main.h"
#include "adc.h"
// #include <cstdint>



void DMA_Cache_PrepareForReceive(void *addr, uint32_t size);

void DMA_Cache_PrepareForSend(void *addr, uint32_t size);


#endif // BSP_CACHE_HPP