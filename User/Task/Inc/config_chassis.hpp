#pragma once

#define CHASSIS_ONLY

#define JUMP_UP
// #define STAIR_UP

#define TOF_DATA_SIZE 9

#define NORMAL_LEG_LEN    0.17f
#define MIN_LEG_LEN       0.15f//true:14
#define MID_LEG_LEN       0.24f
#define MAX_LEG_LEN       0.30f//true:30
#define LQR_LEN_RESOLUTION     0.01f

/* 腿长，单位m */
#define VMC_L1 0.220f
#define VMC_L2 0.260f
#define VMC_MotorDistance 0.0f
#define VMC_HalfMotorDistance (VMC_MotorDistance / 2.0f)

#define WHEEL_RADIUS 0.075f
#define WHEEL_MASS 1.41f
#define WHEEL_DIST 0.43f

#define BODY_MASS 14.0f

#define MAX_HIP_TOR 40.0f
#define MAX_WHEEL_TOR 15.0f

#define LEG_NORMAL_STEP 0.001f
#define LEG_JUMP_STEP 0.01f

#define F_SPRING 250.0f

#define JOINT_FLAT_DELTA 2.71f
#define JOINT_STAIR_DELTA 1.57f
#define JOINT_RECOVER_DELTA 6.28f

#ifndef DEBUG
#define DEBUG
#endif

typedef enum
{
    DONT,
    START,
    EXTENDING,
    INAIR,
    LANDING
}jump_stage_e;

typedef enum
{
    RELAX = 0,
    RECOVER,
    FLATTEN,
    NEUTRAL,
    NORMAL,
    SPIN,
    OFFGROUND,
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