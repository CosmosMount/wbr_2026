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
#include "config_chassis.hpp"

TX_THREAD UIThread;
TX_SEMAPHORE UIThreadSem;
uint8_t UIThreadStack[2048] = {0};
extern uint8_t UIMsg[8];

using namespace Numeric;
uint8_t debug_ui_reset;

[[noreturn]] void UIThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);
    UI ui;

    const char TXT_ALERT[13] = "TURN ON SPIN";
    const char TXT_AIM[4] = "AIM";
    const char TXT_RUNE[5] = "RUNE";
    const char TXT_NORMAL[7] = "NORMAL";
    const char TXT_JUMP[5] = "JUMP";
    const char TXT_STAIR[6] = "STAIR";
    const char TXT_FLY[4] = "FLY";
    
    int8_t LINE_SUPERCAP;
    int8_t LINE_LBASE,LINE_RBASE,LINE_LLEG,LINE_RLEG;
    int8_t LINE_A1,LINE_A2,LINE_A3,LINE_A4;
    int8_t RECT_AIM,RECT_ROBOT,RECT_RUNE;
    int8_t CIRC_AIM,CIRC_AIMER;
    int8_t CIRC_NORMAL,CIRC_JUMP,CIRC_STAIR,CIRC_FLY;
    int8_t FLOAT_V,FLOAT_LEN,FLOAT_DIST;
    int8_t STR_ALERT;
    int8_t ARC_FRONT;
    int8_t INT_TARGET;
    float arc_start_ang = 330, arc_end_ang = 30;
    bool ui_reset = true;

    om_suber_t *referee_suber = om_subscribe(om_find_topic("referee", UINT32_MAX));
    msg_referee_t referee_data{};
    om_suber_t *chassisui_suber = om_subscribe(om_find_topic("chassisui", UINT32_MAX));
    msg_chassisui_t chassisui{};
    om_suber_t *pendulum_suber = om_subscribe(om_find_topic("pendulum", UINT32_MAX));
    msg_pendulum_t pendulum_data{};
    
    for (;;)
    {
        comm_ui_t *gimbal_ui = reinterpret_cast<comm_ui_t*>(&UIMsg);

        om_suber_export(referee_suber, &referee_data, false);
        om_suber_export(chassisui_suber, &chassisui, false);
        om_suber_export(pendulum_suber, &pendulum_data, false);

        debug_ui_reset = gimbal_ui->reset;

        if (gimbal_ui->reset)
        {
            ui.DeleteAll();
            ui_reset = true;
        }
        else if (ui_reset)
        {
            if (referee_data.robot_status.robot_id != 0)
            {
                ui.SetSenderReceiverId(referee_data.robot_status.robot_id, 
                                referee_data.robot_status.robot_id+256);

                /* robot state UI */
                LINE_SUPERCAP = ui.CreateLine(30, UIObjectColor::Yellow, 2, 695, 300, 1155, 300);
                ARC_FRONT = ui.CreateArc(10, UIObjectColor::Yellow, 2, 960, 540, 90, 90, arc_start_ang, arc_end_ang);
                FLOAT_V = ui.CreateFloat(2, UIObjectColor::Yellow, 2, 120, 750, 20, 0);
                FLOAT_LEN = ui.CreateFloat(2, UIObjectColor::Yellow, 2, 120, 800, 20, 0);
                FLOAT_DIST = ui.CreateFloat(2, UIObjectColor::Yellow, 2, 120, 700, 20, 0);
                
                /* aim UI */
                RECT_AIM = ui.CreateRect(3, UIObjectColor::White, 3, 695, 700, 1155, 330);
                RECT_ROBOT = ui.CreateRect(3, UIObjectColor::White, 3, 695, 710, 765, 760);
                RECT_RUNE = ui.CreateRect(3, UIObjectColor::White, 3, 775, 710, 865, 760);
                CIRC_AIM = ui.CreateCircle(3, UIObjectColor::Green, 3, 695, 330, 15);
                CIRC_AIMER = ui.CreateCircle(3, UIObjectColor::Orange, 3, 960, 540, 80);
                INT_TARGET = ui.CreateInt(2, UIObjectColor::Yellow, 3, 710, 685, 15, 0);
                ui.CreateString(2, UIObjectColor::White, 3, 705, 745, 20, TXT_AIM);
                ui.CreateString(2, UIObjectColor::White, 3, 785, 745, 20, TXT_RUNE);

                /* state UI */
                ui.CreateString(3, UIObjectColor::Yellow, 4, 1500, 660, 20, TXT_NORMAL);
                ui.CreateString(3, UIObjectColor::Yellow, 4, 1500, 610, 20, TXT_JUMP);
                ui.CreateString(3, UIObjectColor::Yellow, 4, 1500, 560, 20, TXT_STAIR);
                ui.CreateString(3, UIObjectColor::Yellow, 4, 1500, 510, 20, TXT_FLY);
                CIRC_NORMAL = ui.CreateCircle(3, UIObjectColor::Yellow, 5, 1700, 650, 15);
                CIRC_JUMP = ui.CreateCircle(3, UIObjectColor::Yellow, 5, 1700, 600, 15);
                CIRC_STAIR = ui.CreateCircle(3, UIObjectColor::Yellow, 5, 1700, 550, 15);
                CIRC_FLY = ui.CreateCircle(3, UIObjectColor::Yellow, 5, 1700, 500, 15);

                /* hurt UI */
                STR_ALERT = ui.CreateString(3, UIObjectColor::Team, 5, 800, 800, 25, TXT_ALERT);
                LINE_A1 = ui.CreateLine(3, UIObjectColor::Team, 5, 920, 830, 1000, 830);
                LINE_A2 = ui.CreateLine(3, UIObjectColor::Team, 5, 1300, 500, 1300, 580);
                LINE_A3 = ui.CreateLine(3, UIObjectColor::Team, 5, 920, 250, 1000, 250);
                LINE_A4 = ui.CreateLine(3, UIObjectColor::Team, 5, 620, 500, 620, 580);

                /* leg UI */
                LINE_LBASE = ui.CreateLine(3, UIObjectColor::Yellow, 6, 1450, 800, 1600, 800);
                LINE_RBASE = ui.CreateLine(3, UIObjectColor::Yellow, 6, 1650, 800, 1800, 800);
                LINE_LLEG = ui.CreateLine(3, UIObjectColor::Yellow, 6, 1525, 800, 1525, 750);
                LINE_RLEG = ui.CreateLine(3, UIObjectColor::Yellow, 6, 1725, 800, 1725, 750);

                ui.SetVisible(RECT_RUNE, false);
                ui.SetVisible(RECT_ROBOT, false);
                ui.SetVisible(ARC_FRONT, false);
                ui.SetVisible(CIRC_AIM, false);
                ui.SetVisible(CIRC_NORMAL, false);
                ui.SetVisible(CIRC_JUMP, false);
                ui.SetVisible(CIRC_STAIR, false);
                ui.SetVisible(CIRC_FLY, false);
                ui.SetVisible(STR_ALERT, false);
                ui.SetVisible(LINE_A1, false);
                ui.SetVisible(LINE_A2, false);
                ui.SetVisible(LINE_A3, false);
                ui.SetVisible(LINE_A4, false);
                ui_reset = false;
            }
        }
        else 
        {
            ui.SetVisible(ARC_FRONT, true);
            ui.MoveP2To(LINE_SUPERCAP, 695+SuperCap::Instance()->GetCapEnergy()*0.245, 300);
            
            ui.SetStartAngle(ARC_FRONT, LoopFloatConstrain(arc_start_ang-chassisui.relative_angle*RadToDegree,0.0f,360.0f));
            ui.SetEndAngle(ARC_FRONT, LoopFloatConstrain(arc_end_ang-chassisui.relative_angle*RadToDegree,0.0f,360.0f));

            ui.SetFloat(FLOAT_V, fabs(pendulum_data.v));
            ui.SetFloat(FLOAT_LEN, pendulum_data.len*100.0f);
            ui.SetFloat(FLOAT_DIST, chassisui.dist);

            if (gimbal_ui->aim_rune)
            {
                ui.SetVisible(RECT_RUNE, true);
                ui.SetVisible(RECT_ROBOT, false);
            }
            else 
            {
                ui.SetVisible(RECT_RUNE, false);
                ui.SetVisible(RECT_ROBOT, true);
            }

            ui.SetInt(INT_TARGET, gimbal_ui->aim_target_now);

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

            if (gimbal_ui->bulletfreq)
            {
                ui.SetColor(CIRC_AIMER, UIObjectColor::Orange);
            }
            else 
            {
                ui.SetColor(CIRC_AIMER, UIObjectColor::Yellow);
            }
            
            if (pendulum_data.normal)
                ui.SetVisible(CIRC_NORMAL, true);
            else
                ui.SetVisible(CIRC_NORMAL, false);

            if (chassisui.jumping)
                ui.SetVisible(CIRC_JUMP, true);
            else
                ui.SetVisible(CIRC_JUMP, false);

            if (pendulum_data.stairing)
                ui.SetVisible(CIRC_STAIR, true);
            else
                ui.SetVisible(CIRC_STAIR, false);
            
            if (pendulum_data.flying)
                ui.SetVisible(CIRC_FLY, true);
            else
                ui.SetVisible(CIRC_FLY, false);

            const float pitch = pendulum_data.pitch;
            const float lalpha = pendulum_data.lalpha;
            const float ralpha = pendulum_data.ralpha;

            const int body_x = 1525;
            const int body_y = 800;
            const float half_base = 75.0f;
            const float leg_len = 100.0f;

            const float cos_p = arm_cos_f32(-pitch);
            const float sin_p = arm_sin_f32(-pitch);

            // body base endpoints
            const int lbase_fx = static_cast<int>(body_x + half_base * cos_p);
            const int lbase_fy = static_cast<int>(body_y - half_base * sin_p);
            const int lbase_bx = static_cast<int>(body_x - half_base * cos_p);
            const int lbase_by = static_cast<int>(body_y + half_base * sin_p);
            const int rbase_fx = static_cast<int>(body_x + 200 + half_base * cos_p);
            const int rbase_fy = static_cast<int>(body_y - half_base * sin_p);
            const int rbase_bx = static_cast<int>(body_x + 200 - half_base * cos_p);
            const int rbase_by = static_cast<int>(body_y + half_base * sin_p);

            ui.MoveTo(LINE_LBASE, lbase_fx, lbase_fy);
            ui.MoveP2To(LINE_LBASE, lbase_bx, lbase_by);
            ui.MoveTo(LINE_RBASE, rbase_fx, rbase_fy);
            ui.MoveP2To(LINE_RBASE, rbase_bx, rbase_by);

            // legs
            const float theta_l = pitch + lalpha;
            const float theta_r = pitch + ralpha;

            const int l_end_x = static_cast<int>(body_x + leg_len * arm_sin_f32(theta_l));
            const int l_end_y = static_cast<int>(body_y - leg_len * arm_cos_f32(theta_l));
            const int r_end_x = static_cast<int>(body_x + 200 + leg_len * arm_sin_f32(theta_r));
            const int r_end_y = static_cast<int>(body_y - leg_len * arm_cos_f32(theta_r));

            ui.MoveP2To(LINE_LLEG, l_end_x, l_end_y);
            ui.MoveP2To(LINE_RLEG, r_end_x, r_end_y);

            if (referee_data.robot_hurt.HP_deduction_reason == 0 && referee_data.robot_hurt.armor_id != 0)
            {
                if (referee_data.robot_hurt.armor_id == 1)
                    ui.SetVisible(LINE_A1,true);
                else
                    ui.SetVisible(LINE_A1,false);
                if (referee_data.robot_hurt.armor_id == 2)
                    ui.SetVisible(LINE_A2,true);
                else
                    ui.SetVisible(LINE_A2,false);
                if (referee_data.robot_hurt.armor_id == 3)
                    ui.SetVisible(LINE_A3,true);
                else
                    ui.SetVisible(LINE_A3,false);
                if (referee_data.robot_hurt.armor_id == 4)
                    ui.SetVisible(LINE_A4,true);
                else
                    ui.SetVisible(LINE_A4,false);
                if (!pendulum_data.spinning)
                    ui.SetVisible(STR_ALERT, true);
                else
                    ui.SetVisible(STR_ALERT, false);
            }
            else
            {
                ui.SetVisible(LINE_A1,false);
                ui.SetVisible(LINE_A2,false);
                ui.SetVisible(LINE_A3,false);
                ui.SetVisible(LINE_A4,false);
                ui.SetVisible(STR_ALERT, false);
            }
        }

        ui.Update();
        tx_semaphore_put(&UIThreadSem);
        tx_thread_sleep(100);
    }
}