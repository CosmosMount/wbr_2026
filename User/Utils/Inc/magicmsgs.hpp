#pragma once

#include <cstdint>
typedef enum {
    Relax = 2,
    Spin = 1,
    Normal = 3,
    R2N = 4,
    N2R = 5,
    N2S = 6,
    S2N = 7
}CTRL_STATE;

typedef enum {
    Closed = 2,
    Warm = 3,
    Fire = 1
}SHOOT_STATE;

typedef enum {
    SPD,
    POS,
    TORQUE
}CTRL_MODE;

/**
 * @brief 遥控器消息结构
 */
struct msg_remoter_t 
{
    CTRL_STATE ctrl_sw;
    SHOOT_STATE shoot_sw;
    CTRL_STATE last_ctrl_sw;
    SHOOT_STATE last_shoot_sw;
    float left_x;
    float left_y;
    float right_x;
    float right_y;
    float mouse_x;
    float mouse_y;
    float mouse_z;
    bool mouse_left;
    bool mouse_right;
    
    struct __attribute__((packed)) {
        uint16_t W : 1;
        uint16_t S : 1;
        uint16_t A : 1;
        uint16_t D : 1;
        uint16_t SHIFT : 1;
        uint16_t CTRL : 1;
        uint16_t Q : 1;
        uint16_t E : 1;
        uint16_t R : 1;
        uint16_t F : 1;
        uint16_t G : 1;
        uint16_t Z : 1;
        uint16_t X : 1;
        uint16_t C : 1;
        uint16_t V : 1;
        uint16_t B : 1;
    } key;

    struct __attribute__((packed)) {
       uint16_t W : 1;
       uint16_t S : 1;
       uint16_t A : 1;
       uint16_t D : 1;
       uint16_t SHIFT : 1;
       uint16_t CTRL : 1;
       uint16_t Q : 1;
       uint16_t E : 1;
       uint16_t R : 1;
       uint16_t F : 1;
       uint16_t G : 1;
       uint16_t Z : 1;
       uint16_t X : 1;
       uint16_t C : 1;
       uint16_t V : 1;
       uint16_t B : 1;
    } last_key;

    bool offline;
};

/**
 * @brief AHRS消息结构
 */
struct msg_ins_t {
    float quaternion[4];    ///< 四元数
    float roll;             ///< 横滚角, deg
    float pitch;            ///< 俯仰角, deg
    float yaw;              ///< 偏航角, deg
    float total_yaw;        ///< 偏航总角度, deg
    float gyro_r;           ///< roll角速度, rad/s
    float gyro_p;           ///< pitch角速度, rad/s
    float gyro_y;           ///< yaw角速度, rad/s
    float accel[3];
};

struct msg_solver_t
{
    float llen;
    float llen_dot;
    float rlen;
    float rlen_dot;

    float lphi;
    float lphi_dot;
    float rphi;
    float rphi_dot;
};

struct msg_ctrl_t
{
    float Tl[2];
    float Tr[2];
    float Twl;
    float Twr;
};

struct msg_odometry_t
{
    float x;
    float v;
    float a_z;
};
