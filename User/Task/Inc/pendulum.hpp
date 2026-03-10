#pragma once

#include "fast_math_functions.h"
#include "vmc.hpp"
#include "math.hpp"
#include "odometry.hpp"
#include "arm_math.h"
#include "config_chassis.hpp"

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

    static constexpr float Tk_wheel = 1400.0f;
    static constexpr float wheel_mass = 15.0f;
    static constexpr float max_hip_tor = 40.0f;
    static constexpr float max_wheel_tor = 15.0f;

    float joint1_pos_init;
    float joint4_pos_init;

public:
    Pendulum(bool _reverse, hiptype* _joint1, hiptype* _joint4, wheeltype* _wheel)
        : joint1(_joint1), joint4(_joint4), wheel(_wheel)
    {
        this->reverse = _reverse;
        this->prev_dlen = 0.0f;
        this->neutral_count = 0;
        this->delta_init = false;
        this->dlen = 0.0f;
        this->joint1_pos_init = 0.0f;
        this->joint4_pos_init = 0.0f;
        this->flat = false;
        this->neutral = false;
    }

    float phi;
    float alpha;
    float dalpha;
    float len;
    float dlen;

    float N;

    float alpha_eq;

    bool flat;
    bool neutral;
    bool delta_init;

    void Solve(float _pitch, float _dpitch, float _az)
    {
        /* inverse kinematics */
        /* resolve vmc */
        float joint1_pos = this->joint1->motorFeedback.positionFdb;
        float joint4_pos = this->joint4->motorFeedback.positionFdb;
        float joint1_vel = this->joint1->motorFeedback.speedFdb;
        float joint4_vel = this->joint4->motorFeedback.speedFdb;
        float joint1_tor = this->joint1->motorFeedback.torqueFdb;
        float joint4_tor = this->joint4->motorFeedback.torqueFdb;
        float phi1 = this->reverse ? (PI-joint1_pos) : (PI+joint1_pos);
        float phi4 = this->reverse ? (-joint4_pos) : (joint4_pos);
        float dphi1 = this->reverse ? (-joint1_vel) : (joint1_vel);
        float dphi4 = this->reverse ? (-joint4_vel) : (joint4_vel);

        joint1_tor = this->reverse ? (-joint1_tor) : (joint1_tor);
        joint4_tor = this->reverse ? (-joint4_tor) : (joint4_tor);
        
        this->vmc.Resolve(phi1, phi4);
        /* xdot */
        float qdot[2] = {dphi1, dphi4};
        float xdot[2] = {0.0f, 0.0f};
        this->vmc.VMCVelCal(qdot, xdot);

        /* len params */
        this->len = this->vmc.GetLen();
        this->dlen = xdot[0];

        /* alpha params */
        phi = this->vmc.GetPhi();
        this->alpha = Numeric::LoopFloatConstrain(phi-0.5f*PI+_pitch, -PI, PI);
        this->dalpha = xdot[1] + _dpitch;

        /* inverse dynamics */
        float Treal[2] = {joint1_tor, joint4_tor};
        float Trev[2] = {0.0f, 0.0f};
        this->vmc.VMCRevCal(Trev, Treal);
        float P = (Trev[0])*arm_cos_f32(this->alpha) 
                + Trev[1]/this->len*arm_sin_f32(this->alpha); //+F_SPRING*cos(phi1-phi)
        float ddlen = this->dlen-this->prev_dlen;
        this->N = P + wheel_mass*(_az - ddlen*arm_cos_f32(this->alpha));

        /* neutral and flat detection */
        if (Numeric::abs(this->alpha-this->alpha_eq) < 0.2f)
            this->neutral_count++;
        else
            this->neutral_count = 0;
        this->neutral = this->neutral_count > 200;
        this->flat = (2.5f <= phi && phi <= 3.1f);

        this->prev_dlen = this->dlen;
    }

    void Relax()
    {
        this->neutral = false;
        this->delta_init = false;
        this->joint1->KP = 0.0f; this->joint4->KP = 0.0f;
        this->joint1->KD = 0.0f; this->joint4->KD = 0.0f;
        this->joint1->speedSet = 0.0f; this->joint1->torqueSet = 0.0f;
        this->joint4->speedSet = 0.0f; this->joint4->torqueSet = 0.0f;
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
        this->joint1->torqueSet = 0.0f;
        this->joint4->torqueSet = 0.0f;
        this->joint1->positionSet = joint1_pos_init + (_pdelta * (this->reverse ? -1.0f : 1.0f));
        this->joint4->positionSet = joint4_pos_init + (_pdelta * (this->reverse ? -1.0f : 1.0f));
        this->joint1->speedSet = 0.0f;
        this->joint4->speedSet = 0.0f;
        this->joint1->KP = _kp; this->joint4->KP = _kp;
        this->joint1->KD = _kd; this->joint4->KD = _kd;
    }

    void SpdControl(float _spd, float _kd)
    {
        this->joint1->torqueSet = 0.0f;
        this->joint4->torqueSet = 0.0f;
        this->joint1->speedSet = _spd * (this->reverse ? -1.0f : 1.0f);
        this->joint4->speedSet = _spd * (this->reverse ? -1.0f : 1.0f);
        this->joint1->KP = 0.0f; this->joint4->KP = 0.0f;
        this->joint1->KD = _kd; this->joint4->KD = _kd;
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