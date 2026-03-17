#include "referee.hpp"
#include "string.h"
#include "bsp_dwt.hpp"

void Referee::ProcessData()
{

}

void Referee::HandleMsg(uint8_t *_Msgptr)
{
    uint16_t cmd_id = 0;
    memcpy(&cmd_id, _Msgptr, sizeof(uint16_t));
    _Msgptr += sizeof(uint16_t);
    switch (cmd_id)
    {
    case RefereeID::GameStatus:
        memcpy(&GameStatus, _Msgptr, sizeof(GameStatus));
        break;

    case RefereeID::GameResult:
        memcpy(&GameResult, _Msgptr, sizeof(GameResult));
        break;

    case RefereeID::RobotHP:
        memcpy(&RobotHP, _Msgptr, sizeof(RobotHP));
        break;

    case RefereeID::EventData:
        memcpy(&EventData, _Msgptr, sizeof(EventData));
        break;

    case RefereeID::RefereeWarning:
        memcpy(&RefereeWarning, _Msgptr, sizeof(RefereeWarning));
        break;

    case RefereeID::DartInfo:
        memcpy(&DartInfo, _Msgptr, sizeof(DartInfo));
        break;

    case RefereeID::GameRobotStatus:
        memcpy(&GameRobotStatus, _Msgptr, sizeof(GameRobotStatus));
        GameRobotStatusTick = DWT_GetTimeline_ms();
        break;

    case RefereeID::PowerHeatData:
        memcpy(&PowerHeatData, _Msgptr, sizeof(PowerHeatData));
        PowerHeatTick = DWT_GetTimeline_ms();
        break;

    case RefereeID::GameRobotPos:
        memcpy(&GameRobotPos, _Msgptr, sizeof(GameRobotPos));
        break;

    case RefereeID::Buff:
        memcpy(&Buff, _Msgptr, sizeof(Buff));
        break;

    case RefereeID::RobotHurt:
        memcpy(&RobotHurt, _Msgptr, sizeof(RobotHurt));
        break;

    case RefereeID::ShootData:
        memcpy(&ShootData, _Msgptr, sizeof(ShootData));
        break;

    case RefereeID::RfidStatus:
        memcpy(&RfidStatus, _Msgptr, sizeof(RfidStatus));
        break;

    case RefereeID::DartClientCmd:
        memcpy(&DartClientCmd, _Msgptr, sizeof(DartClientCmd));
        break;

    case RefereeID::RoboInteractData:
        memcpy(&RoboInteractData, _Msgptr, sizeof(RoboInteractData));
        break;

    default:
        break;
    }
}