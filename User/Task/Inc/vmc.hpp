#pragma once

#include "arm_math.h"
#include "config_chassis.hpp"

class cVMCSolver
{
protected:

    /*雅可比矩阵*/
    float J_mat[4]={0};
    float JT_mat[4]={0};
    float JT_inv_mat[4]={0};

    /*关节电机弧度*/
    float phi1 = 0.0f;
    float phi4 = 0.0f;

    /*极限值*/
    float phi1_max = PI;
    float phi4_max = PI / 2;
    float phi1_min = PI / 2;
    float phi4_min = 0.0f;

    /*倒立摆长度*/
    float PendulumLength = 0.0f;
    /*倒立摆角度*/
    float PendulumRadian = PI/2;
    /*倒立摆坐标*/
    float CoorC[2]={0.0f,0.0f};
    /*第二象限节点坐标*/
    float CoorB[2]={0.0f,0.0f};
    float U2 = 0.0f;
    /*第二象限节点坐标*/
    float CoorD[2]={0.0f,0.0f};
    float U3 = 0.0f;


public:

    void Resolve(float phi1_fdb, float phi4_fdb)
    {
        this->phi1 = phi1_fdb;
        this->phi4 = phi4_fdb;       

        float SIN1 = arm_sin_f32(this->phi1);
        float COS1 = arm_cos_f32(this->phi1);
        float SIN4 = arm_sin_f32(this->phi4);
        float COS4 = arm_cos_f32(this->phi4);

        float xdb = VMC_MotorDistance + VMC_L1 * (COS4 - COS1);
        float ydb = VMC_L1 * (SIN4 - SIN1);

        float A0 = 2 * VMC_L2 * xdb;
        float B0 = 2 * VMC_L2 * ydb;
        float C0 = xdb * xdb + ydb * ydb;


        /*计算u2*/
        float u2t = 0.0f;
        arm_atan2_f32((B0 + sqrtf(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0), &u2t);
        this->U2 = 2.0f*(u2t);

        /*计算B*/
        this->CoorB[0] = VMC_L1 * COS1 - VMC_HalfMotorDistance;
        this->CoorB[1] = VMC_L1 * SIN1;

        /*计算C*/
        this->CoorC[0] = this->CoorB[0] + VMC_L2 * arm_cos_f32(this->U2);
        this->CoorC[1] = this->CoorB[1] + VMC_L2 * arm_sin_f32(this->U2);

        /*计算D*/
        this->CoorD[0] = VMC_L1 * COS4 + VMC_HalfMotorDistance;
        this->CoorD[1] = VMC_L1 * SIN4;
        /*计算u3*/
        float u3t;
        arm_atan2_f32((this->CoorD[1] - this->CoorC[1]), (this->CoorD[0] - this->CoorC[0]), &u3t);
        this->U3 = PI+u3t;

        /*计算倒立摆长度*/
        arm_atan2_f32(this->CoorC[1], this->CoorC[0], &this->PendulumRadian);
        this->PendulumLength = sqrtf(this->CoorC[0] * this->CoorC[0] + this->CoorC[1] * this->CoorC[1]);

        /*计算J与J^T*R*M*/
        float sin32 = arm_sin_f32(this->U3 - this->U2);
        float sin12 = arm_sin_f32(this->phi1 - this->U2);
        float sin34 = arm_sin_f32(this->U3 - this->phi4);
        float cos03 = arm_cos_f32(this->PendulumRadian - this->U3);
        float cos02 = arm_cos_f32(this->PendulumRadian - this->U2);
        float sin03 = arm_sin_f32(this->PendulumRadian - this->U3);
        float sin02 = arm_sin_f32(this->PendulumRadian - this->U2);

        J_mat[0] = VMC_L1 * sin03 * sin12 / sin32;
        J_mat[1] = VMC_L1 * sin02 * sin34 / sin32;
        J_mat[2] = VMC_L1 * cos03 * sin12 / (sin32 * PendulumLength);
        J_mat[3] = VMC_L1 * cos02 * sin34 / (sin32 * PendulumLength);

        JT_mat[0] = VMC_L1 * sin03 * sin12 / sin32;
        JT_mat[1] = VMC_L1 * cos03 * sin12 / (sin32 * PendulumLength);
        JT_mat[2] = VMC_L1 * sin02 * sin34 / sin32;
        JT_mat[3] = VMC_L1 * cos02 * sin34 / (sin32 * PendulumLength);

        JT_inv_mat[0] = -cos02 / (sin12 * VMC_L1);
        JT_inv_mat[1] = cos03 / (sin34 * VMC_L1);
        JT_inv_mat[2] = sin02 * PendulumLength / (sin12 * VMC_L1);
        JT_inv_mat[3] = -sin03 * PendulumLength / (sin34 * VMC_L1);
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

    inline float GetPendulumLen() {
        return PendulumLength;
    }

    inline float GetPendulumRadian() {
        return PendulumRadian;
    }

    inline float GetPhi4() {
        return phi4;
    }

    inline float GetPhi1() {
        return phi1;
    }

    inline void GetPendulumCoor(float* Coor) {
        Coor[0]=this->CoorC[0];Coor[1]=this->CoorC[1];
    }
    inline void GetCoorB(float* Coor) {
        Coor[0]=this->CoorB[0];Coor[1]=this->CoorB[1];
    }
    inline void GetCoorD(float*Coor) {
        Coor[0]=this->CoorD[0];Coor[1]=this->CoorD[1];
    }
};
