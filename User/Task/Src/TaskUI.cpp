#include "bsp_dwt.hpp"
#include "om.h"
#include "main.h"
#include "tx_api.h"
#include "ui.hpp"
#include <cstdint>

TX_THREAD UIThread;
uint8_t UIThreadStack[2048] = {0};

[[noreturn]] void UIThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);
    UI ui;
    ui.SetSenderReceiverId(1, 0x0101);
    char TXT_AUTO[] = "AUTO";
    ui.CreateString(3, UIObjectColor::Cyan, 1, 1542, 860, 20, TXT_AUTO);
    // ui.CreateString(3, UIObjectColor::Cyan, 2, 1542, 820, 20, TXT_OUTPOST);
    // ui.CreateString(3, UIObjectColor::Cyan, 2, 1542, 780, 20, TXT_HERO);
    // ui.CreateString(3, UIObjectColor::Cyan, 2, 1542, 740, 20, TXT_ENGINEER);
    // ui.CreateString(3, UIObjectColor::Cyan, 2, 1542, 700, 20, TXT_INFANTRY3);
    // ui.CreateString(3, UIObjectColor::Cyan, 2, 1542, 660, 20, TXT_INFANTRY4);
    // ui.CreateString(3, UIObjectColor::Cyan, 2, 1542, 620, 20, TXT_INFANTRY5);
    // ui.CreateString(3, UIObjectColor::Cyan, 2, 1542, 580, 20, TXT_SENTRY);
    // ui.CreateString(3, UIObjectColor::Cyan, 2, 1542, 540, 20, TXT_RUNE);
    // ui.CreateString(3, UIObjectColor::Cyan, 2, 1542, 500, 20, TXT_BASE);
    // ui.CreateString(3, UIObjectColor::Cyan, 2, 1542, 460, 20, TXT_NONE);

    ui.CreateLine(3, UIObjectColor::Pink, 2, 582, 440, 682, 543);
    ui.CreateLine(3, UIObjectColor::Pink, 3, 1087, 544, 1317, 387);
    int8_t LINE_SUPERCAP = ui.CreateLine(30, UIObjectColor::Yellow, 4, 800, 100, 1175, 100);
    for (;;)
    {
        ui.MoveP2To(LINE_SUPERCAP, 800+DWT_GetTimeline_ms(), 100);
        ui.Update();
        tx_thread_sleep(100);
    }
}