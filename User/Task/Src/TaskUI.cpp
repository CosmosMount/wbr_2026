#include "bsp_dwt.hpp"
#include "om.h"
#include "main.h"
#include "tx_api.h"
#include "ui.hpp"
#include "math.hpp"
#include "config_comm.hpp"
#include "magicmsgs.hpp"
#include "supercap.hpp"
#include "config_referee.hpp"

TX_THREAD UIThread;
TX_SEMAPHORE UIThreadSem;
uint8_t UIThreadStack[2048] = {0};
extern uint8_t UIMsg[8];

using namespace Numeric;

[[noreturn]] void UIThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);
    UI ui;

    // const char TXT_OUTPOST[8] = "OUTPOST";
    // const char TXT_HERO[5] = "HERO";
    // const char TXT_ENGINEER[10] = "ENGINEER";
    // const char TXT_INFANTRY3[10] = "INFANTRY3";
    // const char TXT_INFANTRY4[10] = "INFANTRY4";
    // const char TXT_INFANTRY5[10] = "INFANTRY5";
    // const char TXT_SENTRY[7] = "SENTRY";
    // const char TXT_BASE[5] = "BASE";
    // const char TXT_RUNE[5] = "RUNE";
    // const char TXT_AUTO[5] = "AUTO";
    // const char TXT_NONE[5] = "NONE";
    const char TXT_ALERT[13] = "TURN ON SPIN";
    const char TXT_VELOCITY[9] = "VELOCITY";
    
    int8_t LINE_SUPERCAP;
    int8_t RECT_AIM;
    int8_t CIRC_AIM,CIRC_AIMER;
    int8_t FLOAT_V,FLOAT_LEN;
    int8_t STR_ALERT;
    int8_t ARC_FRONT;
    float arc_start_ang = 330, arc_end_ang = 30;
    bool ui_reset = true;

    om_suber_t *referee_suber = om_subscribe(om_find_topic("referee", UINT32_MAX));
    msg_referee_t referee_data{};
    om_suber_t *chassisui_suber = om_subscribe(om_find_topic("chassisui", UINT32_MAX));
    msg_chassisui_t chassisui{};
    
    for (;;)
    {
        comm_ui_t *gimbal_ui = reinterpret_cast<comm_ui_t*>(&UIMsg);

        // om_suber_export(cmd_suber, &cmd, false);
        om_suber_export(referee_suber, &referee_data, false);
        om_suber_export(chassisui_suber, &chassisui, false);

        if (gimbal_ui->reset)
        {
            ui.DeleteAll();
            ui_reset = true;
        }
        else if (ui_reset)
        {
            if (referee_data.robot_status.robot_id != 0)
            {
                // ui.SetSenderReceiverId(1, 0x0101);
                ui.SetSenderReceiverId(referee_data.robot_status.robot_id, 
                                referee_data.robot_status.robot_id+256);

                // ui.CreateString(3, UIObjectColor::Cyan, 1, 1542, 860, 20, TXT_AUTO);
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
                // ui.CreateLine(3, UIObjectColor::Pink, 2, 582, 440, 682, 543);
                // ui.CreateLine(3, UIObjectColor::Pink, 3, 1087, 544, 1317, 387);

                // ui.CreateString(3, UIObjectColor::Black, 1, 750, 180, 25, TXT_VELOCITY);

                LINE_SUPERCAP = ui.CreateLine(30, UIObjectColor::Yellow, 2, 800, 300, 1175, 300);
                RECT_AIM = ui.CreateRect(3, UIObjectColor::White, 3, 695, 700, 1155, 330);
                CIRC_AIM = ui.CreateCircle(3, UIObjectColor::Green, 4, 695, 330, 15);
                CIRC_AIMER = ui.CreateCircle(3, UIObjectColor::Orange, 4, 960, 540, 80);
                ARC_FRONT = ui.CreateArc(10, UIObjectColor::Yellow, 4, 960, 540, 90, 90, arc_start_ang, arc_end_ang);
                FLOAT_V = ui.CreateFloat(2, UIObjectColor::Yellow, 3, 120, 700, 30, 0);
                FLOAT_LEN = ui.CreateFloat(2, UIObjectColor::Yellow, 3, 120, 800, 30, 0);

                ui.SetVisible(LINE_SUPERCAP, false);
                ui.SetVisible(ARC_FRONT, false);
                ui.SetVisible(CIRC_AIM, false);
                ui_reset = false;
            }
        }
        else 
        {
            ui.SetVisible(ARC_FRONT, true);
            ui.SetVisible(LINE_SUPERCAP, true);
            // ui.MoveP2To(LINE_SUPERCAP, 800+SuperCap::Instance()->GetCapEnergy() * 0.2, 100);
            
            ui.SetStartAngle(ARC_FRONT, LoopFloatConstrain(arc_start_ang+chassisui.relative_angle*RadToDegree,0.0f,360.0f));
            ui.SetEndAngle(ARC_FRONT, LoopFloatConstrain(arc_end_ang+chassisui.relative_angle*RadToDegree,0.0f,360.0f));

            ui.SetFloat(FLOAT_V, chassisui.v);
            ui.SetFloat(FLOAT_LEN, chassisui.len);

            if (gimbal_ui->aim_target_now == 0)
            {
                ui.SetColor(RECT_AIM, UIObjectColor::White);
                ui.SetVisible(CIRC_AIM, false);
            }
            else 
            {
                ui.SetColor(RECT_AIM, UIObjectColor::Magenta);
                ui.SetVisible(CIRC_AIM, true);
                ui.MoveTo(CIRC_AIM, 695 + gimbal_ui->aim_target_x * 4.6f, 330 + gimbal_ui->aim_target_y * 3.7f);
            }

            if (gimbal_ui->fire)
            {
                ui.SetColor(CIRC_AIMER, UIObjectColor::Team);
            }
            else 
            {
                ui.SetColor(CIRC_AIMER, UIObjectColor::Yellow);
            }
        }

        ui.Update();
        tx_semaphore_put(&UIThreadSem);
        tx_thread_sleep(100);
    }
}