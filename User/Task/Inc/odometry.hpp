# pragma once

#include "math.hpp"
#include "kalman_filter.h"
#include "magicmsgs.hpp"

#define t  0.001f
#define t2 0.000001f
#define t3 0.000000001f
#define t4 0.000000000001f
#define t5 0.000000000000001f

class VelFusionKF
{
protected:
    const float qq = 5.0f;//10
    const float rv = 0.1f;
    const float ra = 50.0f;//25.0f

    const float A_Init[9] = {1, t, t2 / 2, 0, 1, t, 0, 0, 1};
    const float Q_Init[9] = {t5 / 20 * qq, t4 / 8 * qq, t3 / 6 * qq, t4 / 8 * qq, t3 / 3 * qq, t2 / 2 * qq, t3 / 6 * qq,
                             t2 / 2 * qq, t * qq};
    const float H_Init[6] = {0, 1, 0, 0, 0, 1};
    const float P_Init[9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
    const float R_Init[4] = {rv, 0, 0, ra};

public:
    KalmanFilter_t KF;

    VelFusionKF()
    {
        Kalman_Filter_Init(&this->KF, 3, 0, 2);//Inertia odome 3 State 2 observation
        memcpy(this->KF.P_data, P_Init, sizeof(P_Init));
        memcpy(this->KF.F_data, A_Init, sizeof(A_Init));
        memcpy(this->KF.Q_data, Q_Init, sizeof(Q_Init));
        memcpy(this->KF.H_data, H_Init, sizeof(H_Init));
        memcpy(this->KF.R_data, R_Init, sizeof(R_Init));
    }

    void ResetKF(KalmanFilter_t *kf)
    {
        memset(kf->xhat.pData, 0, sizeof(float) * kf->xhat.numRows);
        memset(kf->xhatminus.pData, 0, sizeof(float) * kf->xhatminus.numRows);
        memset(kf->P.pData, 0, sizeof(float) * kf->P.numRows*kf->P.numRows);
    }

    void UpdateKalman(float Velocity, float AccelerationX)
    {
        this->KF.MeasuredVector[0] = Velocity;
        this->KF.MeasuredVector[1] = AccelerationX;
        Kalman_Filter_Update(&this->KF);
    }

    float GetXhat()
    {
        return this->KF.xhat.pData[0];
    }

    float GetVhat()
    {
        return this->KF.xhat.pData[1];
    }

};

class Odometry
{
private:
    msg_odometry_t odom_data_;
    VelFusionKF vel_kf;
public:

    /**
     * @brief odometry update function
     * @note all the params must be homography
     * @param _quaternion [w, x, y, z] format
     * @param _acc [0, ax, ay, az] quaternion format for acceleration
     * @param _vel velocity measurement
     * @param _yaw in degree
     * @return odometry_info
     */
    msg_odometry_t Update(float *_quaternion, float *_acc, float _vel, float _yaw)
    {
        odom_data_.x = 0.0f;
        odom_data_.v = 0.0f;
        odom_data_.a_z = 0.0f;

        float temp[4] = {0};
        float a_world[4] = {0};

        arm_quaternion_product_f32(_quaternion, _acc, temp, 1);
        arm_quaternion_product_f32(temp, _quaternion, a_world, 1);

        float a_x = sqrtf(a_world[1] * a_world[1] + a_world[2] * a_world[2]) *
                    arm_cos_f32(atan2f(a_world[2], a_world[1]) - _yaw * Numeric::DegreeToRad);

        vel_kf.UpdateKalman(_vel, a_x);

        odom_data_.v = vel_kf.GetVhat();
        odom_data_.x = vel_kf.GetXhat();
        odom_data_.a_z = a_world[3];

        return odom_data_;
    }

    void Reset()
    {
        vel_kf.ResetKF(&vel_kf.KF);
        odom_data_ = {0.0f, 0.0f, 0.0f};
    }
};
