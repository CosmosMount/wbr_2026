#include "bsp_cache.hpp"


void DMA_Cache_PrepareForSend(void *addr, uint32_t size)
{
    // uint32_t address = (uint32_t)addr;
    // uint32_t length = size;

    // //地址需要32字节对齐
    // address &= ~(uint32_t)0x1F;
    // length = (length + 31) & ~(uint32_t)0x1F;

    SCB_CleanDCache_by_Addr((uint32_t *)addr, size); // 清除数据缓存
}


void DMA_Cache_PrepareForReceive(void *addr, uint32_t size)
{
    // uint32_t address = (uint32_t)addr;
    // uint32_t length = size;

    // // 地址32字节对齐
    // address &= ~(uint32_t)0x1F;
    // // 长度补齐到32字节对齐
    // length = (length + 31) & ~(uint32_t)0x1F;

    SCB_InvalidateDCache_by_Addr((uint32_t *)addr, size);

}



