#pragma once

#include <cstdint>
#include <string>
#include "stdint.h"
#include "string.h"
#include "crc.hpp"
#include "usart.h"
#include "dma.h"
#include "adc.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

enum class UIObjectType : uint8_t
{
    Line,
    Rect,
    Circle,
    Ellipse,
    Arc,
    Float,
    Int,
    Str,
};

enum class UIObjectColor : uint8_t
{
    Team,
    Yellow,
    Green,
    Orange,
    Magenta,
    Pink,
    Cyan,
    Black,
    White,
};

enum class UIOperation : uint8_t
{
    Noop,
    Add,
    Modify,
    Delete,
};

enum class UIIDErrorCode : int32_t
{
    NoMoreSpace = -1,
    MutexTimeout = -2,
};

class UI
{
public:
    // Set up sender/receiver IDs
    void SetSenderReceiverId(uint16_t senderId, uint16_t receiverId);

    // Object creation functions
    int8_t CreateLine(int width, UIObjectColor color, int layer, int x1, int y1, int x2, int y2);
    int8_t CreateRect(int width, UIObjectColor color, int layer, int x1, int y1, int x2, int y2);
    int8_t CreateCircle(int width, UIObjectColor color, int layer, int x, int y, int radius);
    int8_t CreateEllipse(int width, UIObjectColor color, int layer, int x, int y, int xSemiaxis, int ySemiaxis);
    int8_t CreateArc(int width, UIObjectColor color, int layer, int x, int y, int xSemiaxis, int ySemiaxis, int startAngle, int endAngle);
    int8_t CreateFloat(int width, UIObjectColor color, int layer, int x, int y, int fontSize, float value);
    int8_t CreateInt(int width, UIObjectColor color, int layer, int x, int y, int fontSize, int value);
    int8_t CreateString(int width, UIObjectColor color, int layer, int x, int y, int fontSize, const char* str);

    // Modifying object properties
    void SetVisible(int id, bool visible);
    void SetColor(int id, UIObjectColor color);
    void SetWidth(int id, int width);
    void SetFontSize(int id, int fontSize);
    void SetStringChanged(int id);
    void MoveTo(int id, int x, int y);
    void MoveP2To(int id, int x, int y);
    void SetRadius(int id, int radius);
    void SetSemiaxis(int id, int xSemiaxis, int ySemiaxis);
    void SetStartAngle(int id, int startAngle);
    void SetEndAngle(int id, int endAngle);
    void SetFloat(int id, float value);
    void SetInt(int id, int value);
    void SetString(int id, const char* str);

    // Deletion functions
    void Delete(int id);
    void DeleteAll();
    void DeleteLayer(int layer);

    void Update();

private:

    struct UITxFrameHeader
    {
        uint8_t SOF;
        uint16_t DataLength;
        uint8_t Seq;
        uint8_t Crc8;
        uint16_t CommandId;
        uint16_t ContentId;
        uint16_t SenderId;
        uint16_t ReceiverId;
    } __attribute__((packed));

    struct UIObject
    {
        struct UIObjectMetadata
        {
            bool valid : 1;
            bool deleted : 1;
            bool dirty : 1;
            bool dirtyVisibility : 1;
            bool visible : 1;
        } __attribute__((packed)) metadata;

        uint8_t refereeHandle[3];

        union DetailDword1Internals
        {
            uint32_t dw;
            struct
            {
                uint32_t operation : 3;
                uint32_t type : 3;
                uint32_t layer : 4;
                uint32_t color : 4;
                uint32_t detailA : 9;
                uint32_t detailB : 9;
            } __attribute__((packed));
        } detailDword1 __attribute__((packed));

        union DetailDword2Internals
        {
            uint32_t dw;
            struct
            {
                uint32_t width : 10;
                uint32_t x : 11;
                uint32_t y : 11;
            } __attribute__((packed));
        } detailDword2 __attribute__((packed));

        union DetailDword3Internals
        {
            uint32_t dw;
            struct
            {
                uint32_t radius : 10;
                uint32_t reserved : 22;
            } circle __attribute__((packed));
            struct
            {
                uint32_t reserved : 10;
                uint32_t x2 : 11;
                uint32_t y2 : 11;
            } line __attribute__((packed));
            struct
            {
                uint32_t reserved : 10;
                uint32_t xSemiaxis : 11;
                uint32_t ySemiaxis : 11;
            } ellipse __attribute__((packed));
            int intVal;
            uint32_t floatVal;
            const char* strVal;
        } detailDword3 __attribute__((packed));
    } __attribute__((packed));

    struct OfficialUIObject
    {
        char name[3];
        union Dword1Union
        {
            uint32_t detailDword1;
            struct
            {
                uint32_t operation : 3;
                uint32_t not_used : 29;
            } __attribute__((packed)) detailDword1Internal;
        } __attribute__((packed)) Dword1;
        uint32_t detailDword2;
        uint32_t detailDword3;
    } __attribute__((packed));

    struct UIDelete
    {
        uint8_t Type;
        uint8_t Layer;
    } __attribute__((packed));

    static constexpr uint8_t UI_TOTAL_COUNT = 30;
    static constexpr uint8_t STRING_MAX_LENGTH = 30;
    static constexpr uint8_t TX_BUFFER_SIZE = 120;
    static constexpr uint8_t MAX_STRING_PER_FRAME = 2;
    static constexpr uint8_t MAX_OTHER_PER_FRAME = 7;

    static UIObject UIObjectList[UI_TOTAL_COUNT];
    static uint8_t UITxBuffer[TX_BUFFER_SIZE];
    static UIDelete UIDeleteOp;

    uint8_t IncreaseID = 0;
    bool UIPendingUpdateIsString = false;
    uint8_t UIPendingStringIndex = 0;
    uint8_t UIScanOffset = 0;

    static constexpr uint8_t elementCountInPacketTable[8] = {0, 1, 2, 5, 5, 5, 7, 7};
    static constexpr uint16_t contentIdTable[8] = {0, 0x0101, 0x0102, 0x0103, 0x0103, 0x0103, 0x0104, 0x0104};

    inline UITxFrameHeader* getFrameHeader() 
    {
        return reinterpret_cast<UITxFrameHeader*>(UITxBuffer);
    }

    inline OfficialUIObject* getBufferNthUiObject(uint8_t index) 
    {
        return reinterpret_cast<OfficialUIObject*>(UITxBuffer + sizeof(UITxFrameHeader) + index * sizeof(OfficialUIObject));
    }

    inline char* getBufferStringBuffer() 
    {
        return reinterpret_cast<char*>(UITxBuffer + sizeof(UITxFrameHeader) + sizeof(OfficialUIObject));
    }

    template<typename Callback>
    void loopScanObjectList(uint8_t fromIndex, uint8_t untilIndex, Callback cb) 
    {
        uint8_t upperBound = (untilIndex <= fromIndex) ? (untilIndex + UI_TOTAL_COUNT) : untilIndex;
        for (uint8_t i = fromIndex; i < upperBound; ++i) 
        {
            if (!cb(i % UI_TOTAL_COUNT, UIObjectList[i % UI_TOTAL_COUNT])) 
                break;
        }
    }

    inline void loopIncrement(uint8_t& x) { x = (x + 1) % UI_TOTAL_COUNT; }

    inline int8_t CreateAndInitObject() 
    {
        for (uint8_t i = 0; i < UI_TOTAL_COUNT; i++) 
        {
            auto& obj = UIObjectList[i];
            if (!obj.metadata.valid) 
            {
                obj.metadata.valid = true;
                obj.metadata.deleted = false;
                obj.metadata.dirty = false;
                obj.metadata.dirtyVisibility = true;
                obj.metadata.visible = true;
                memcpy(obj.refereeHandle, (void *)&IncreaseID, 3);
                obj.detailDword1.dw = 0;
                obj.detailDword2.dw = 0;
                obj.detailDword3.dw = 0;
                IncreaseID++;
                return static_cast<int8_t>(i);
            }
        }
        return -1;
    }

    inline void SendData(uint8_t* data, uint16_t len)
    {
        SCB_CleanDCache_by_Addr((uint32_t *)data, len);
        HAL_UART_Transmit_DMA(&huart1, data, len);
    }

    void TransmitStringObject(uint8_t index, UIOperation op);
    void TransmitOtherObjects(uint8_t count);
};
