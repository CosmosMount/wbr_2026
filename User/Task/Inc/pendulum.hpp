#pragma once

#include "fast_math_functions.h"
#include "vmc.hpp"
#include "math.hpp"
#include "odometry.hpp"
#include "arm_math.h"
#include "config_chassis.hpp"

// ---------------------------------------------------------------------------
// Schraudolph fast exp  (~4 ULP, ~5 cycles on Cortex-M7)
// ---------------------------------------------------------------------------
__attribute__((always_inline, optimize("O3")))
static inline float _fast_expf(float x) noexcept
{
    if (x >  88.3f) return 3.4028235e+38f;
    if (x < -87.3f) return 0.0f;
    union { float f; int32_t i; } u;
    u.i = static_cast<int32_t>(12102203.161561485f * x) + 1065353216;
    return u.f;
}

template <typename hiptype, typename wheeltype>
class Pendulum
{
protected:
    VMCsolver vmc;
    uint32_t neutral_count;
    float prev_dlen;
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

    // -----------------------------------------------------------------------
    // 气弹簧 Freal 两项模型
    //
    // 拟合目标：Freal（纯蹲起工况，全程接地数据）
    //
    //   Freal_model(len, dlen) = F_static(len) + k_d · dlen
    //
    // 【F_static 项】衰减正弦，由示波器图像中位数轨迹拟合：
    //   F_static = (A·exp(-α·dx) + C_env)·sin(ω·dx + φ) + DC0 + DC1·dx
    //   dx = len - x0，α/ω 为左右腿对称共享参数
    //
    // 【k_d·dlen 项】线性阻尼，由图像包络半宽反推：
    //   包络半宽均值 ≈ 23 N，k_d = 23 / dlen_max
    //   ⚠ dlen_max 未知，默认假设 ±0.3 m/s → k_d ≈ 76.7 N/(m/s)
    //   上机后调用 calibrate_kd(observed_dlen_max) 修正：
    //     dlen_max = 0.2 m/s → k_d ≈ 115
    //     dlen_max = 0.3 m/s → k_d ≈  77  ← 默认
    //     dlen_max = 0.5 m/s → k_d ≈  46
    //
    // 【离地检测原理】
    //   拟合数据全部来自接地蹲起，模型捕捉的是接地状态下的 Freal 基线。
    //   残差 F_delta = Freal - Freal_model：
    //     接地：F_delta ≈ 0（气弹簧力已被模型解释）
    //     离地：关节力矩卸载，Freal 突降，F_delta << 0
    //   相比直接判断 Freal > thresh，残差消除了 len/dlen 的非线性影响，
    //   阈值是与腿长无关的固定值，整定更直观。
    // -----------------------------------------------------------------------

    struct LegParams {
        float x0;     ///< 摆长原点 [m]
        float A;      ///< 衰减振幅 [N]
        float C_env;  ///< 包络基线 [N]
        float phi;    ///< 相位 [rad]
        float DC0;    ///< DC 偏置 [N]
        float DC1;    ///< DC 斜率 [N/m]
    };

    static constexpr float FIT_ALPHA = 25.0000f;   ///< 共享衰减率
    static constexpr float FIT_OMEGA = 42.9043f;   ///< 共享角频率 [rad/m]

    static constexpr LegParams kLeft = {
        .x0    =  0.15f,
        .A     =  50.000f,
        .C_env =  -4.103f,
        .phi   =  -0.0865f,
        .DC0   =   3.11f,
        .DC1   = -259.71f,
    };
    static constexpr LegParams kRight = {
        .x0    =  0.14f,
        .A     =  12.310f,
        .C_env =   1.158f,
        .phi   =   1.0858f,
        .DC0   = -10.39f,
        .DC1   = -238.70f,
    };

    __attribute__((always_inline, optimize("O3")))
    static float Freal_model_eval(float len, float dlen,
                                  const LegParams& p, float kd) noexcept
    {
        const float dx  = len - p.x0;
        const float env = p.A * _fast_expf(-FIT_ALPHA * dx) + p.C_env;
        return env * arm_sin_f32(FIT_OMEGA * dx + p.phi)
               + p.DC0 + p.DC1 * dx
               + kd * dlen;
    }

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
        this->neutral_count   = 0;
        this->delta_init      = false;
        this->dlen            = 0.0f;
        this->joint1_pos_init = 0.0f;
        this->joint4_pos_init = 0.0f;
        this->flat            = false;
        this->neutral         = false;
        this->airborne        = false;
        this->liftoff_count   = 0;
        this->landing_count   = 0;
    }

    float phi;
    float alpha;
    float dalpha;
    float len;
    float dlen;
    float N;
    float alpha_eq;

    float Freal;         ///< VMC 逆动力学实测值 [N]
    float Freal_model;   ///< 气弹簧模型预测值 [N]：F_static(len) + k_d·dlen
    float F_delta;       ///< 残差 = Freal - Freal_model
                         ///<   接地 ≈ 0，离地 << 0（关节卸载）

    bool flat;
    bool neutral;
    bool delta_init;
    bool airborne;       ///< 离地状态（经防抖确认）

    // ── 可在线调整的参数 ───────────────────────────────────────────────────

    /// 阻尼系数 [N/(m/s)]，默认假设 dlen_max=0.3 m/s
    /// 上机后调用 calibrate_kd() 修正
    float kd = 76.7f;

    /// 离地检测：F_delta 低于此值判定为离地候选 [N]
    /// 建议整定：静止站立时观察 |F_delta| 噪声基底，设为其 3 倍左右
    float liftoff_thresh = -60.0f;

    /// 离地确认帧数（1kHz 控制频率下，10 = 10ms）
    /// 过小易误触发，过大延迟大；跳跃场景建议 5~10
    uint32_t liftoff_confirm = 1;

    /// 落地确认帧数：落地冲击响应要快，建议 3~5
    uint32_t landing_confirm = 1;

    float alpha_eq_coeff[3] = { 0.280918f, -1.101757f, 1.232768f };

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
        this->alpha    = Numeric::LoopFloatConstrain(phi - 0.5f*PI + _pitch, -PI, PI);
        this->dalpha   = xdot[1] + _dpitch;
        this->alpha_eq = this->alpha_eq_coeff[0]
                       + this->alpha_eq_coeff[1] * this->len
                       + this->alpha_eq_coeff[2] * this->len * this->len;

        if (this->len < 0.17f)
            this->alpha_eq *= 0.75f;

        /* ── 逆动力学 ────────────────────────────────────────────────────── */
        float Treal[2] = {joint1_tor, joint4_tor};
        float Trev[2]  = {0.0f, 0.0f};
        this->vmc.VMCRevCal(Trev, Treal);

        float P     = Trev[0] * arm_cos_f32(this->alpha)
                    + Trev[1] / this->len * arm_sin_f32(this->alpha);
        float ddlen = this->dlen - this->prev_dlen;
        this->N     = P + wheel_mass * (_az - ddlen * arm_cos_f32(this->alpha));
        this->Freal = Trev[0];

        /* ── 气弹簧模型 & 残差 ───────────────────────────────────────────── */
        const LegParams& lp  = this->reverse ? kRight : kLeft;
        this->Freal_model    = Freal_model_eval(this->len, this->dlen, lp, this->kd);
        this->F_delta        = this->Freal - this->Freal_model;
        //  接地正常蹲起：F_delta ≈ 0
        //  离地关节卸载：F_delta << 0（幅值 > liftoff_thresh 的绝对值）

        /* ── 离地检测（带滞后防抖）──────────────────────────────────────── */
        if (!this->airborne)
        {
            // 离地候选：残差持续低于阈值
            if (this->F_delta < this->liftoff_thresh )
                this->liftoff_count++;
            else
                this->liftoff_count = 0;

            if (this->liftoff_count >= this->liftoff_confirm)
            {
                this->airborne      = true;
                this->liftoff_count = 0;
            }
        }
        else
        {
            // 落地确认：残差回归零附近（关节重新承载）
            // 注意：落地冲击时 F_delta 会短暂正跳，用 > liftoff_thresh/2 即可
            if (this->F_delta > this->liftoff_thresh * 0.5f)
                this->landing_count++;
            else
                this->landing_count = 0;

            if (this->landing_count >= this->landing_confirm)
            {
                this->airborne      = false;
                this->landing_count = 0;
            }
        }

        /* ── neutral & flat 检测 ─────────────────────────────────────────── */
        if (Numeric::abs(this->alpha - this->alpha_eq) < 0.2f)
            this->neutral_count++;
        else
            this->neutral_count = 0;
        this->neutral = this->neutral_count > 200;
        this->flat    = (2.5f <= phi && phi <= 3.1f);

        this->prev_dlen = this->dlen;
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

    void DeltaPControl(float _pdelta, float _kp, float _kd)
    {
        if (!delta_init)
        {
            joint1_pos_init = this->joint1->motorFeedback.positionFdb;
            joint4_pos_init = this->joint4->motorFeedback.positionFdb;
            delta_init = true;
        }
        this->joint1->torqueSet   = 0.0f;
        this->joint4->torqueSet   = 0.0f;
        this->joint1->positionSet = joint1_pos_init + (_pdelta * (this->reverse ? -1.0f : 1.0f));
        this->joint4->positionSet = joint4_pos_init + (_pdelta * (this->reverse ? -1.0f : 1.0f));
        this->joint1->speedSet    = 0.0f;
        this->joint4->speedSet    = 0.0f;
        this->joint1->KP = _kp; this->joint4->KP = _kp;
        this->joint1->KD = _kd; this->joint4->KD = _kd;
    }

    void SpdControl(float _spd, float _kd)
    {
        this->joint1->torqueSet  = 0.0f;
        this->joint4->torqueSet  = 0.0f;
        this->joint1->speedSet   = _spd * (this->reverse ? -1.0f : 1.0f);
        this->joint4->speedSet   = _spd * (this->reverse ? -1.0f : 1.0f);
        this->joint1->KP = 0.0f; this->joint4->KP = 0.0f;
        this->joint1->KD = _kd;  this->joint4->KD = _kd;
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