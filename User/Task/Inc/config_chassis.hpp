#pragma once

#define SJTU_MODEL
#define CHASSIS_ONLY

#define TOF_DATA_SIZE 9

#define LQR_MIN_LEN_CTRL       0.15f
#define LQR_MAX_LEN_CTRL       0.35f
#define LQR_LEN_RESOLUTION     0.01f

/*腿长，单位m*/
#define VMC_L1 0.150f
#define VMC_L2 0.270f
#define VMC_MotorDistance 0.150f
#define VMC_HalfMotorDistance (VMC_MotorDistance / 2.0f)

#define LJOINT4_OFFSET 0x11FF
#define LJOINT1_OFFSET 0x25FF
#define RJOINT4_OFFSET 0xC6C3
#define RJOINT1_OFFSET 0x8419

#define WHEEL_RADIUS 0.077f
#define WHEEL_MASS 1.41f

#define MAX_HIP_TOR 40.0f
#define MAX_WHEEL_TOR 15.0f

#ifndef DEBUG
#define DEBUG
#endif

typedef enum{
    ON_GROUND = 0,
    OFF_GROUND = 1,
} fly_flag_e;

/*底盘运动模式*/
typedef enum
{
    NONE = 0,
    NORMAL_MOVING_MODE,
    ESCAPE_MODE,
    ABNORMAL_MOVING_MODE
} chassis_mode_e;

typedef enum
{
    NORMAL_ROTATE = 0,
    SPIN_ROTATE
} rotate_ctrl_e;

typedef enum
{
    DO_NOT_JUMP = 0,
    JUMP_READY,
    JUMP_START
} jump_ctrl_e;

typedef enum
{
    NOT_FLY_MODE = 0,
    FLY_MODE
} fly_ctrl_e;

#pragma pack(push,1)

typedef struct
{
    chassis_mode_e chassis_mode : 2;
    rotate_ctrl_e rotate_type : 1;
    jump_ctrl_e jump_ctrl : 2;
    fly_ctrl_e fly_ctrl : 1;
} chassis_mode_t;

struct tof_data_t
{
    uint8_t header[2];
    uint16_t distance;
    uint16_t strength;
    uint16_t temp_raw;
    uint8_t check_sum;
};

#pragma pack(pop)