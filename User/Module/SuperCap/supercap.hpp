#pragma once

#include "main.h"
#include <cstring>
#include "fdcan.h"
#include "bsp_can.hpp"

struct supercap_set_t
{
    uint8_t cap_state_set;
    uint8_t fly_extra_set;
    float power_limit_set;
    uint8_t raw_power_limit;
    uint8_t remained_energy;
} __attribute__((packed));

struct supercap_fdb_t
{
    uint8_t cap_state_fdb;
    uint8_t cap_voltage_x5;
    int16_t input_power_x100;
    int16_t cap_power_x100;
    int16_t remained_energy;
} __attribute__((packed));

union supercap_set_u
{
    supercap_set_t set;
    uint8_t _data[sizeof(supercap_set_t)];
};

union supercap_fdb_u
{
    supercap_fdb_t fdb;
    uint8_t _data[sizeof(supercap_fdb_t)];
};

class SuperCap
{
public:
    supercap_set_u supercap_set; ///< 用于发送给超级电容的数据
    supercap_fdb_u supercap_fdb; ///< 用于接收超级电容的数据

    typedef enum : uint8_t
    {
        SuperCapOFF = 0,
        SuperCapON = 1,
    } SuperCapState;

    void Init();
    // 设置超级电容的状态
    void SetCapState(SuperCapState state);
    void SetFlyExtra(uint8_t extra);
    void SetPowerLimit(float power);
    // 获取超级电容的状态
    SuperCapState GetCapState();
    float GetCapVoltage();
    float GetInputPower();
    float GetCapPower();
    float GetCapInPower();
    float GetCapEnergy();
    void SendCapData();
    void ReceiveCapData(uint8_t *data, uint16_t len);

    static SuperCap* Instance()
    {
        static SuperCap instance;
        return &instance;
    }
};
