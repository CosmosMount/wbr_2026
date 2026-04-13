#pragma once

#include "arm_math.h"
#include "config_chassis.hpp"

namespace chassis
{
class VMCsolver
{
protected:

    /* Jacobian */
    float J_mat[4]={0};
    float JT_mat[4]={0};
    float JT_inv_mat[4]={0};

    /* Joint1,4 Angles */
    float phi1 = 0.0f;
    float phi4 = 0.0f;

    /* Inverted Pendulum Length */
    float len = 0.0f;
    /* Inverted Pendulum Angle */
    float phi = PI/2;
    /* Pendulum Angle */
    float alpha = 0.0f;
    /* C */
    float CoorC[2]={0.0f,0.0f};
    /* Joint2 Coordinates */
    float CoorB[2]={0.0f,0.0f};
    float U2 = 0.0f;
    /* Joint3 Coordinates */
    float CoorD[2]={0.0f,0.0f};
    float U3 = 0.0f;

    /* Spring Force */
    float Fs = 0.0f;

public:

    void Resolve(float _phi1, float _phi4)
    {
        this->phi1 = _phi1;
        this->phi4 = _phi4;       

        float SIN1 = arm_sin_f32(this->phi1);
        float COS1 = arm_cos_f32(this->phi1);
        float SIN4 = arm_sin_f32(this->phi4);
        float COS4 = arm_cos_f32(this->phi4);

        float xdb = L1 * (COS4 - COS1);
        float ydb = L1 * (SIN4 - SIN1);

        float A0 = 2 * L2 * xdb;
        float B0 = 2 * L2 * ydb;
        float C0 = xdb * xdb + ydb * ydb;


        /* 计算u2 */
        float u2t = 0.0f;
        arm_atan2_f32((B0 + sqrtf(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0), &u2t);
        this->U2 = 2.0f*(u2t);

        /* 计算B坐标 */
        this->CoorB[0] = L1 * COS1;
        this->CoorB[1] = L1 * SIN1;

        /* 计算C坐标 */
        this->CoorC[0] = this->CoorB[0] + L2 * arm_cos_f32(this->U2);
        this->CoorC[1] = this->CoorB[1] + L2 * arm_sin_f32(this->U2);

        /* 计算D坐标 */
        this->CoorD[0] = L1 * COS4;
        this->CoorD[1] = L1 * SIN4;

        /* 计算u3 */
        float u3t;
        arm_atan2_f32((this->CoorD[1] - this->CoorC[1]), (this->CoorD[0] - this->CoorC[0]), &u3t);
        this->U3 = PI+u3t;

        /* 计算倒立摆长度 */
        arm_atan2_f32(this->CoorC[1], this->CoorC[0], &this->phi);
        this->len = sqrtf(this->CoorC[0] * this->CoorC[0] + this->CoorC[1] * this->CoorC[1]);

        /* 计算J与J^T*R*M */
        float sin32 = arm_sin_f32(this->U3 - this->U2);
        float sin12 = arm_sin_f32(this->phi1 - this->U2);
        float sin34 = arm_sin_f32(this->U3 - this->phi4);
        float cos03 = arm_cos_f32(this->phi - this->U3);
        float cos02 = arm_cos_f32(this->phi - this->U2);
        float sin03 = arm_sin_f32(this->phi - this->U3);
        float sin02 = arm_sin_f32(this->phi - this->U2);

        J_mat[0] = L1 * sin03 * sin12 / sin32;
        J_mat[1] = L1 * sin02 * sin34 / sin32;
        J_mat[2] = L1 * cos03 * sin12 / (sin32 * len);
        J_mat[3] = L1 * cos02 * sin34 / (sin32 * len);

        JT_mat[0] = L1 * sin03 * sin12 / sin32;
        JT_mat[1] = L1 * cos03 * sin12 / (sin32 * len);
        JT_mat[2] = L1 * sin02 * sin34 / sin32;
        JT_mat[3] = L1 * cos02 * sin34 / (sin32 * len);

        JT_inv_mat[0] = -cos02 / (sin12 * L1);
        JT_inv_mat[1] = cos03 / (sin34 * L1);
        JT_inv_mat[2] = sin02 * len / (sin12 * L1);
        JT_inv_mat[3] = -sin03 * len / (sin34 * L1);

        /* 计算弹簧等效力 */
        // 1. 基础角度计算 (只调用一次三角函数)
        float theta = PI - this->phi1 + this->U2;
        float sin_theta = arm_sin_f32(theta);
        float cos_theta = arm_cos_f32(theta);
        float gemma = this->phi1 - Ang_spring;

        // 2. 坐标计算，相对于 L1
        // 足端点Y坐标 (用于推导雅可比)
        float Yc = L2 * sin_theta; 
        
        // 气弹簧活动端坐标 (小腿上)
        float Xh = -L1 + Dspring2 * cos_theta;
        float Yh = Dspring2 * sin_theta;
        
        // 气弹簧固定端坐标
        float X0 = -Dspring1 * arm_cos_f32(gemma); 
        float Y0 = Dspring1 * arm_sin_f32(gemma);

        // 3. 计算真实的气弹簧长度
        float Ls = sqrtf((Xh - X0) * (Xh - X0) + (Yh - Y0) * (Yh - Y0));
        if (Ls < 1e-4f) Ls = 1e-4f;

        // 4. 雅可比速度映射
        float denom = 2.0f * L1 * Yc; 
        float numer = 2.0f * (Yh * (X0 + L1) - Y0 * (Xh + L1));

        // 5. 虚功等效力计算 (Fv = Fs * TR)
        // 防除零保护
        if (denom > 1e-4f || denom < -1e-4f) 
        {
            this->Fs = Fspring * numer * this->len / (denom * Ls);
        } 
        else 
        {
            this->Fs = 0.0f;
        }
    }

    void VMCCal(float *F, float *T)
    {
        T[0] = this->JT_mat[0] * F[0] + this->JT_mat[1] * F[1];
        T[1] = this->JT_mat[2] * F[0] + this->JT_mat[3] * F[1];
    }

    void VMCRevCal(float *F, float *T)
    {
        F[0] = this->JT_inv_mat[0] * T[0] + this->JT_inv_mat[1] * T[1];
        F[1] = this->JT_inv_mat[2] * T[0] + this->JT_inv_mat[3] * T[1];
    }

    void VMCVelCal(float *phi_dot, float *v_dot)
    {
        v_dot[0] = this->J_mat[0] * phi_dot[0] + this->J_mat[1] * phi_dot[1];
        v_dot[1] = this->J_mat[2] * phi_dot[0] + this->J_mat[3] * phi_dot[1];
    }

    inline float GetLen() {    return len;    }

    inline float GetPhi() {    return phi;    }

    inline float GetPhi4() {    return phi4;    }

    inline float GetPhi1() {    return phi1;    }

    inline float GetFs() {    return Fs;    }
};
} // namespace chassis

