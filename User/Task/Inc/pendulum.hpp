#pragma once

#include "vmc.hpp"
#include "math.hpp"
#include "odometry.hpp"
#include "arm_math.h"
#include "config_chassis.hpp"

namespace chassis
{

template <typename hiptype, typename wheeltype>
class Pendulum
{
protected:
    VMCsolver vmc;
    uint32_t neutral_count;
    float prev_dlen;
    float prev_dalpha;
    bool reverse;

    hiptype*   joint1;
    hiptype*   joint4;
    wheeltype* wheel;

    static constexpr float Tk_wheel      = 1400.0f;
    static constexpr float wheel_mass    = 15.0f;
    static constexpr float max_hip_tor   = 40.0f;
    static constexpr float max_wheel_tor = 15.0f;

    float joint1_pos_init;
    float joint4_pos_init;

    // ── 离地检测内部状态 ────────────────────────────────────────────────────
    uint32_t liftoff_count = 0;   ///< 连续离地投票计数
    uint32_t landing_count = 0;   ///< 连续落地投票计数

    constexpr static float dlen_stable_threshold = 0.10f; ///< 离地检测时的 dlen 稳定性阈值

public:

    Pendulum(bool _reverse, hiptype* _joint1, hiptype* _joint4, wheeltype* _wheel)
        : joint1(_joint1), joint4(_joint4), wheel(_wheel)
    {
        this->reverse         = _reverse;
        this->prev_dlen       = 0.0f;
        this->prev_dalpha     = 0.0f;
        this->neutral_count   = 0;
        this->delta_init      = false;
        this->dlen            = 0.0f;
        this->joint1_pos_init = 0.0f;
        this->joint4_pos_init = 0.0f;
        this->flat            = false;
        this->neutral         = false;
        this->liftoff_count   = 0;
        this->landing_count   = 0;
    }

    float phi;
    float alpha;
    float dalpha;
    float alpha_eq;

    float len;
    float dlen;

    float N;
    float Fs;
    float Freal;
    float Treal;

    bool flat;
    bool neutral;
    bool delta_init = false;

    PID len_pd = PID(6000.0f, 0.0f, -900.0f, 125.0f, 0.0f, PID_DVEL);
    PID phi_pd = PID(0.7f, 0.0f, 1.4f, 10.0f, 0.005f);

    SLOPE phi_updater = SLOPE(0.0f, 0.005f);

    void Solve(float _pitch, float _dpitch, float _az)
    {
        /* ── 逆运动学 & VMC ─────────────────────────────────────────────── */
        float joint1_pos = this->joint1->motorFeedback.positionFdb;
        float joint4_pos = this->joint4->motorFeedback.positionFdb;
        float joint1_vel = this->joint1->motorFeedback.speedFdb;
        float joint4_vel = this->joint4->motorFeedback.speedFdb;
        float joint1_tor = this->joint1->motorFeedback.torqueFdb;
        float joint4_tor = this->joint4->motorFeedback.torqueFdb;

        float phi1  = this->reverse ? (PI - joint1_pos) : (PI + joint1_pos);
        float phi4  = this->reverse ? (-joint4_pos)     : (joint4_pos);
        float dphi1 = this->reverse ? (-joint1_vel)     : (joint1_vel);
        float dphi4 = this->reverse ? (-joint4_vel)     : (joint4_vel);
        joint1_tor  = this->reverse ? (-joint1_tor)     : (joint1_tor);
        joint4_tor  = this->reverse ? (-joint4_tor)     : (joint4_tor);

        this->vmc.Resolve(phi1, phi4);

        float qdot[2] = {dphi1, dphi4};
        float xdot[2] = {0.0f, 0.0f};
        this->vmc.VMCVelCal(qdot, xdot);

        /* ── 摆长 ───────────────────────────────────────────────────────── */
        this->len  = this->vmc.GetLen();
        this->dlen = xdot[0];

        /* ── 摆角 ───────────────────────────────────────────────────────── */
        phi = this->vmc.GetPhi();
        if (phi < 0.0f)
            phi += 2.0f * PI; // 将 phi 规范到 [0, 2π] 范围内，方便后续判断
        this->alpha    = Numeric::LoopFloatConstrain(phi - 0.5f*PI + _pitch, -PI, PI);
        this->dalpha   = xdot[1] + _dpitch;
        this->alpha_eq = alpha_eq_coeff[0]
                       + alpha_eq_coeff[1] * this->len
                       + alpha_eq_coeff[2] * this->len * this->len;

        /* ── 逆动力学 ────────────────────────────────────────────────────── */
        float Treal[2] = {joint1_tor, joint4_tor};
        float Trev[2]  = {0.0f, 0.0f};
        this->vmc.VMCRevCal(Trev, Treal);
        this->Freal = Trev[0];
        this->Treal = Trev[1];
        this->Fs = this->vmc.GetFs();
        float cos_alpha = arm_cos_f32(this->alpha);
        float sin_alpha = arm_sin_f32(this->alpha);
        float P  = (Trev[0]+this->Fs) * cos_alpha
                  + Trev[1] / this->len * sin_alpha;
        float ddlen   = this->dlen - this->prev_dlen;
        float ddalpha = this->dalpha - this->prev_dalpha;

        float Zw = (_az - Numeric::Gravity)
                - ddlen * cos_alpha
                + 2.0f * this->dlen * this->dalpha * sin_alpha
                + this->len * ddalpha * sin_alpha
                + this->len * this->dalpha * this->dalpha * cos_alpha;

        this->N = P + wheel_mass * Zw;

        /* ── neutral & flat 检测 ─────────────────────────────────────────── */
        if (Numeric::abs(this->alpha - this->alpha_eq) < 0.2f)
            this->neutral_count++;
        else
            this->neutral_count = 0;
        this->neutral = this->neutral_count > 150;
        this->flat    = (2.5f <= phi && phi <= 3.1f);

        this->prev_dlen = this->dlen;
        this->prev_dalpha = this->dalpha;
    }

    void Relax()
    {
        this->neutral    = false;
        this->delta_init = false;
        this->joint1->KP = 0.0f; this->joint4->KP = 0.0f;
        this->joint1->KD = 0.0f; this->joint4->KD = 0.0f;
        this->joint1->speedSet  = 0.0f; this->joint1->torqueSet  = 0.0f;
        this->joint4->speedSet  = 0.0f; this->joint4->torqueSet  = 0.0f;
        this->wheel->currentSet = 0.0f;
    }

    void DeltaPhiControl(float _phi, float _kp, float _kd, float _slope)
    {
        if (!this->delta_init)
        {
            this->phi_updater.SetDefault(this->phi);
            this->phi_updater.SetPath(_slope);
            this->delta_init = true;
        }

        this->phi_pd.Tuning(_kp, 0.0f, _kd);
        this->phi_pd.ref = this->phi_updater.UpdateVal(_phi);
        this->phi_pd.fdb = this->phi;
        this->phi_pd.UpdateResult(this->dalpha);
        float F[2] = {0.0f, 0.0f};
        F[0] = -this->vmc.GetFs();
        F[1] = this->phi_pd.result;
        TorqueControl(F, 0.0f);
    }

    float LenControl(float _ref)
    {
        this->len_pd.ref = _ref;
        this->len_pd.fdb = this->len;
        this->len_pd.UpdateResult(this->dlen);
        return this->len_pd.result - Gff;
    }

    void TorqueControl(float *_F, float _Tw)
    {
        float T[2] = {0.0f, 0.0f};
        this->vmc.VMCCal(_F, T);
        this->joint1->torqueSet = Numeric::FloatConstrain(T[0], -max_hip_tor, max_hip_tor)
                                 * (this->reverse ? -1.0f : 1.0f);
        this->joint4->torqueSet = Numeric::FloatConstrain(T[1], -max_hip_tor, max_hip_tor)
                                 * (this->reverse ? -1.0f : 1.0f);
        this->wheel->currentSet = Numeric::FloatConstrain(_Tw, -max_wheel_tor, max_wheel_tor)
                                 * Tk_wheel * (this->reverse ? 1.0f : -1.0f);
    }
};

};