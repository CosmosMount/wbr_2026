#pragma once

#ifndef DEBUG
#define DEBUG
#endif

// #define GIMBAL_ONLY

#define JUMP_UP
// #define STAIR_UP

#define TOF_DATA_SIZE 9

typedef enum
{
    DONT,
    START,
    EXTENDING,
    INAIR,
    LANDING,
    BACK
} jump_stage_e;

typedef enum
{
    NONE,
    AIR,
    AIRLAND
} air_stage_e;

typedef enum
{
    RELAX = 0,
    RECOVER,
    FLATTEN,
    NEUTRAL,
    NORMAL,
    OFFGROUND,
    SPIN,
    GOSTAIR,
    JUMP
} chassis_state_e;

#pragma pack(push,1)

struct tof_data_t
{
    uint8_t header[2];
    uint16_t distance;
    uint16_t strength;
    uint16_t temp_raw;
    uint8_t check_sum;
};

#pragma pack(pop)

namespace chassis
{

constexpr float L1 = 0.220f;
constexpr float L2 = 0.260f;
constexpr float Lnormal = 0.16f;
constexpr float Lmin = 0.16f;
constexpr float Lmid = 0.24f;
constexpr float Lmax = 0.36f;
constexpr float Lqr_len_resolution = 0.01f;
constexpr float Lswitch = 0.21f;

constexpr float Rwheel = 0.06f;
constexpr float Mwheel = 0.21f;
constexpr float Dwheel = 0.43f;
constexpr float Mbody = 14.0f;

constexpr float Tk_wheel = 1400.0f;
constexpr float Thip_max = 40.0f;
constexpr float Twheel_max = 15.0f;

constexpr float Fspring = 450.0f;
constexpr float Dspring1 = 0.03f;
constexpr float Dspring2 = 0.05f;
constexpr float Ang_spring = 0.2164f;

constexpr float Gff = 67.865f;

constexpr float Nliftoff = 5.0f;
constexpr float Nlanding = 30.0f;

constexpr float pitch_eq = 0.005f;
constexpr float alpha_eq_coeff[3] = { 0.280918f, -1.101757f, 1.232768f };

constexpr float imu_offset_x = 0.2f;

constexpr uint8_t ljoint1_id = 0x01;
constexpr uint8_t ljoint4_id = 0x02;
constexpr uint8_t rjoint1_id = 0x04;
constexpr uint8_t rjoint4_id = 0x03;
constexpr uint16_t lwheel_id = 0x202;
constexpr uint16_t rwheel_id = 0x201;

constexpr float yaw_offset1 = 0.40389235f;
constexpr float yaw_offset2 = -2.7377723f;

};