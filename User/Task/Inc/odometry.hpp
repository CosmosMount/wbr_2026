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
    const float qq = 5.0f;//10
    const float rv = 0.1f;
    const float ra = 50.0f;//25.0f

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
        float _quaternion_conj[4] = {_quaternion[0], -_quaternion[1], -_quaternion[2], -_quaternion[3]};
        arm_quaternion_product_f32(_quaternion, _acc, temp, 1);
        arm_quaternion_product_f32(temp, _quaternion_conj, a_world, 1);

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
        vel_kf.ResetKF();
        odom_data_ = {0.0f, 0.0f, 0.0f};
    }
};
