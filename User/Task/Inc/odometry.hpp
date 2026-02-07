# pragma once

#include "math.hpp"
#include "kalmanfilter.hpp"
#include "magicmsgs.hpp"
#include <cstring>

using namespace Filter;

#define dt  0.001f
#define dt2 0.000001f
#define dt3 0.000000001f
#define dt4 0.000000000001f
#define dt5 0.000000000000001f

class VelFusionKF : public KalmanFilter
{
protected:
    uint32_t ErrorCount = 0;
    bool ConvergeFlag = true;
    const float ChiSquareTestThreshold = 6.0f;
    const float qq = 5.0f;
    const float rv = 0.1f;
    const float ra = 50.0f;

    const float A_Init[9] = {1, dt, dt2 / 2, 0, 1, dt, 0, 0, 1};
    const float Q_Init[9] = {dt3 / 20 * qq, dt4 / 8 * qq, dt3 / 6 * qq, 
                            dt4 / 8 * qq, dt3 / 3 * qq, dt2 / 2 * qq, 
                            dt3 / 6 * qq, dt2 / 2 * qq, dt * qq};
    const float H_Init[6] = {0, 1, 0, 0, 0, 1};
    const float P_Init[9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
    const float R_Init[4] = {rv, 0, 0, ra};

public:
    VelFusionKF() : KalmanFilter(3, 0, 2)
    {
        std::memcpy(this->F_data, A_Init, sizeof(A_Init));
        std::memcpy(this->H_data, H_Init, sizeof(H_Init));
        std::memcpy(this->Q_data, Q_Init, sizeof(Q_Init));
        std::memcpy(this->R_data, R_Init, sizeof(R_Init));
        std::memcpy(this->P_data, P_Init, sizeof(P_Init));
    }

    void ResetKF()
    {
        std::memset(this->xhat_data, 0, sizeof(float) * this->xhatSize);
        std::memset(this->xhatminus_data, 0, sizeof(float) * this->xhatSize);
        std::memcpy(this->P_data, P_Init, sizeof(P_Init));
    }

    void UpdateKalman(float Velocity, float AccelerationX)
    {
        // // 1. 获取预测观测值 y_pred = H * x_minus
        // // H_Init = {0, 1, 0,  0, 0, 1} -> 预测速度是 xhatminus[1], 预测加速度是 xhatminus[2]
        // float v_pred = this->xhatminus_data[1];
        // float a_pred = this->xhatminus_data[2];

        // // 2. 计算残差 (Innovation)
        // float dy = Velocity - v_pred;
        // float da = AccelerationX - a_pred;

        // // 3. 计算残差协方差 S = H * P_minus * H' + R
        // // 针对你的 H 矩阵，S 是 P 矩阵右下角 2x2 块与 R 的和
        // // P 矩阵索引 (3x3): 0:p11, 1:p12, 2:p13, 3:p21, 4:p22, 5:p23, 6:p31, 7:p32, 8:p33
        // float S11 = this->P_data[4] + this->R_data[0]; // Velocity covariance + rv
        // float S12 = this->P_data[5];                   // P_va
        // float S21 = this->P_data[7];                   // P_av
        // float S22 = this->P_data[8] + this->R_data[3]; // Accel covariance + ra

        // // 4. 计算 S 的逆矩阵 det(S) 和 S^-1
        // float detS = S11 * S22 - S12 * S21;
        
        // if (std::abs(detS) > 1e-9f) 
        // {
        //     float invS11 = S22 / detS;
        //     float invS12 = -S12 / detS;
        //     float invS21 = -S21 / detS;
        //     float invS22 = S11 / detS;

        //     // 5. 计算马氏距离平方 d^2 = [dy, da] * S^-1 * [dy, da]'
        //     float chiSquare = dy * (invS11 * dy + invS12 * da) + 
        //                da * (invS21 * dy + invS22 * da);

        //     // 6. 离群点检测逻辑
        //     if (chiSquare > ChiSquareTestThreshold) 
        //     {
        //         ErrorCount++;
        //         if (ErrorCount > 50) {
        //             // 系统彻底发散，可能需要重置里程计或停止控制
        //             ConvergeFlag = false; 
        //         } else {
        //             // 撞击瞬间：拒绝本次更新，保持上一时刻状态
        //             // 相当于直接 Return，里程计位置 x 不会因为这次异常速度而改变
        //             return; 
        //         }
        //     } 
        //     else 
        //     {
        //         // 3. 自适应区间
        //         if (chiSquare > 0.1f * ChiSquareTestThreshold) 
        //         {
        //             // 软抑制：计算一个 0~1 的系数
        //             float scale = (ChiSquareTestThreshold - chiSquare) / (0.9f * ChiSquareTestThreshold);
        //             // 方案：通过调大 R 来间接实现 Adaptive Gain
        //             // R_new = R_old / scale (scale 越小，R 越大)
        //             this->R_data[0] = rv / (scale + 0.01f); 
        //         } 
        //         else 
        //         {
        //             this->R_data[0] = rv;
        //             ErrorCount = 0;
        //         }
                
        //         // 4. 执行标准更新
        //         this->MeasuredVector[0] = Velocity;
        //         this->MeasuredVector[1] = AccelerationX;
        //         this->Update();
        //     }
        // }
        this->MeasuredVector[0] = Velocity;
        this->MeasuredVector[1] = AccelerationX;
        this->Update();
    }

    float GetXhat()
    {
        return this->xhat.pData[0];
    }

    float GetVhat()
    {
        return this->xhat.pData[1];
    }

};

class Odometry
{
private:
    VelFusionKF vel_kf;
public:
    float x;
    float v;
    float az;

    /**
     * @brief odometry update function
     * @note all the params must be homography
     * @param _quaternion [w, x, y, z] format
     * @param _acc [0, ax, ay, az] quaternion format for acceleration
     * @param _vel velocity measurement
     * @param _yaw in radians
     * @return odometry_info
     */
    void Update(float *_quaternion, float *_acc, float _vel, float _yaw)
    {
        x = 0.0f;
        v = 0.0f;
        az = 0.0f;

        float temp[4] = {0};
        float a_world[4] = {0};
        float _quaternion_conj[4] = {_quaternion[0], -_quaternion[1], -_quaternion[2], -_quaternion[3]};
        arm_quaternion_product_f32(_quaternion, _acc, temp, 1);
        arm_quaternion_product_f32(temp, _quaternion_conj, a_world, 1);

        float a_x = sqrtf(a_world[1] * a_world[1] + a_world[2] * a_world[2]) *
                    arm_cos_f32(atan2f(a_world[2], a_world[1]) - _yaw);

        vel_kf.UpdateKalman(_vel, a_x);

        v = vel_kf.GetVhat();
        x = vel_kf.GetXhat();
        az = a_world[3];
    }

    void Reset()
    {
        vel_kf.ResetKF();
        x = 0.0f;
        v = 0.0f;
    }
};
