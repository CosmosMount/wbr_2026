#include "supercap.hpp"
#include "bsp_can.hpp"
#include "fdcan.h"

void SuperCap::Init()
{
    supercap_set.set.cap_state_set = SuperCapON;
    supercap_set.set.fly_extra_set = 0;
    supercap_set.set.power_limit_set = 45;
    supercap_set.set.raw_power_limit = 0;
    supercap_set.set.remained_energy = 0;

    supercap_fdb.fdb.cap_state_fdb = 0;
    supercap_fdb.fdb.cap_voltage_x5 = 0;
    supercap_fdb.fdb.input_power_x100 = 0;
    supercap_fdb.fdb.cap_power_x100 = 0;
    supercap_fdb.fdb.remained_energy = 0;
}

void SuperCap::SetCapState(SuperCapState state)
{
    supercap_set.set.cap_state_set = state;
}

void SuperCap::SetFlyExtra(uint8_t extra)
{
    supercap_set.set.fly_extra_set = extra;
}

void SuperCap::SetPowerLimit(float power)
{
    supercap_set.set.power_limit_set = power;
}

void SuperCap::SendCapData()
{
    CAN_Transmit(&hfdcan3, 0xC5, supercap_set._data, 8);
}

SuperCap::SuperCapState SuperCap::GetCapState()
{
    return static_cast<SuperCapState>(supercap_fdb.fdb.cap_state_fdb);
}

float SuperCap::GetCapVoltage()
{
    return supercap_fdb.fdb.cap_voltage_x5 * 0.2f;
}

float SuperCap::GetInputPower()
{
    return supercap_fdb.fdb.input_power_x100 * 0.01f;
}

float SuperCap::GetCapPower()
{
    return supercap_fdb.fdb.cap_power_x100 * 0.01f;
}

float SuperCap::GetCapEnergy()
{
    return supercap_fdb.fdb.cap_voltage_x5*supercap_fdb.fdb.cap_voltage_x5*0.1111111111f;
}
