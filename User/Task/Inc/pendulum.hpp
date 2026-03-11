#pragma once

#include "fast_math_functions.h"
#include "vmc.hpp"
#include "math.hpp"
#include "odometry.hpp"
#include "arm_math.h"
#include "config_chassis.hpp"

// ---------------------------------------------------------------------------
// Schraudolph fast exp (单精度, ~4 ULP误差, ~5周期 on Cortex-M7)
// CMSIS-DSP 没有 arm_exp_f32，用此替代 expf()
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

    hiptype* joint1;
    hiptype* joint4;
    wheeltype* wheel;

    static constexpr float Tk_wheel    = 1400.0f;
    static constexpr float wheel_mass  = 15.0f;
    static constexpr float max_hip_tor = 40.0f;
    static constexpr float max_wheel_tor = 15.0f;

    float joint1_pos_init;
    float joint4_pos_init;

    // -----------------------------------------------------------------------
    // 拟合模型参数（对称约束，共享 α & ω）
    // 模型: y = (A·exp(-α·dx) + C_env)·sin(ω·dx + φ) + DC0 + DC1·dx
    // 其中 dx = len - x0  （len 即 VMC 输出的摆长）
    // -----------------------------------------------------------------------
    struct LegParams {
        float x0;     ///< 摆长原点（拟合起始值）
        float A;      ///< 衰减振幅
        float C_env;  ///< 包络基线
        float phi;    ///< 相位 [rad]
        float DC0;    ///< DC 偏置
        float DC1;    ///< DC 斜率
    };

    // 共享衰减率 & 角频率
    static constexpr float FIT_ALPHA = 25.0000f;
    static constexpr float FIT_OMEGA = 42.9043f;

    // 左腿参数（对应 llen / FIreal）
    static constexpr LegParams kLeft = {
        .x0    =  0.15f,
        .A     =  50.000f,
        .C_env =  -4.103f,
        .phi   =  -0.0865f,
        .DC0   =   3.11f,
        .DC1   = -259.71f,
    };

    // 右腿参数（对应 rlen / Frreal）
    static constexpr LegParams kRight = {
        .x0    =  0.14f,
        .A     =  12.310f,
        .C_env =   1.158f,
        .phi   =   1.0858f,
        .DC0   = -10.39f,
        .DC1   = -238.70f,
    };

    // -----------------------------------------------------------------------
    // 单腿模型求值：给定当前摆长 len，返回预测 Freal
    // -----------------------------------------------------------------------
    __attribute__((always_inline, optimize("O3")))
    static float EvalLegModel(float len, const LegParams& p) noexcept
    {
        const float dx  = len - p.x0;
        const float env = p.A * _fast_expf(-FIT_ALPHA * dx) + p.C_env;
        const float val = env * arm_sin_f32(FIT_OMEGA * dx + p.phi)
                        + p.DC0 + p.DC1 * dx;
        return val;
    }

public:
    Pendulum(bool _reverse, hiptype* _joint1, hiptype* _joint4, wheeltype* _wheel)
        : joint1(_joint1), joint4(_joint4), wheel(_wheel)
    {
        this->reverse       = _reverse;
        this->prev_dlen     = 0.0f;
        this->neutral_count = 0;
        this->delta_init    = false;
        this->dlen          = 0.0f;
        this->joint1_pos_init = 0.0f;
        this->joint4_pos_init = 0.0f;
        this->flat          = false;
        this->neutral       = false;
    }

    float phi;
    float alpha;
    float dalpha;
    float len;
    float dlen;

    float N;

    float alpha_eq;
    float Freal;         ///< 逆动力学计算的实际 F（原有）
    float Freal_model;   ///< 拟合模型预测的 F（新增，用于对比/融合）
    float Freal_fused;   ///< 融合后的 F（加权平均）

    bool flat;
    bool neutral;
    bool delta_init;

    /* alpha_eq = a1 + a2*len + a3*len^2 */
    float alpha_eq_coeff[3] = { 0.280918f, -1.101757f, 1.232768f };

    // 融合权重：0.0 = 完全使用逆动力学, 1.0 = 完全使用模型
    // 建议先设为 0.3 观察效果，再根据实际情况调整
    float model_weight = 0.3f;

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

        /* ── 逆动力学：原始 Freal & N ────────────────────────────────────── */
        float Treal[2] = {joint1_tor, joint4_tor};
        float Trev[2]  = {0.0f, 0.0f};
        this->vmc.VMCRevCal(Trev, Treal);

        float P     = Trev[0] * arm_cos_f32(this->alpha)
                    + Trev[1] / this->len * arm_sin_f32(this->alpha);
        float ddlen = this->dlen - this->prev_dlen;
        this->N     = P + wheel_mass * (_az - ddlen * arm_cos_f32(this->alpha));

        // 原始逆动力学 Freal
        this->Freal = Trev[0];

        /* ── 拟合模型预测 Freal ──────────────────────────────────────────── */
        // 根据 reverse 选择对应腿的参数
        this->Freal_model = EvalLegModel(
            this->len,
            this->reverse ? kRight : kLeft
        );

        /* ── 加权融合 ────────────────────────────────────────────────────── */
        // Freal_fused 可直接替换下游控制器中原来的 Freal
        // 逆动力学在传感器噪声大时容易抖动，模型值作为平滑先验
        this->Freal_fused = (1.0f - this->model_weight) * this->Freal
                          +         this->model_weight  * this->Freal_model;

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