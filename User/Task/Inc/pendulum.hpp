#pragma once

#include "fast_math_functions.h"
#include "vmc.hpp"
#include "math.hpp"
#include "odometry.hpp"
#include "arm_math.h"
#include "config_chassis.hpp"

class PendulumSolver
{
protected:
    VMCsolver vmc;
    uint32_t neutral_count;
    float prev_dlen;
    bool reverse;

public:
    PendulumSolver(bool _reverse)
    {
        this->reverse = _reverse;
    }

    float alpha;
    float dalpha;
    float len;
    float dlen;

    float N;

    bool flat;
    bool neutral;

    void Update(float _ang1, float _ang4, float _pitch, float _vel1, float _vel4, float _dpitch, float _tor1, float _tor4, float _az)
    {
        /* inverse kinematics */
        /* resolve vmc */
        float _phi1 = this->reverse ? (PI-_ang1) : (PI+_ang1);
        float _phi4 = this->reverse ? (-_ang4) : (_ang4);
        float _dphi1 = this->reverse ? (-_vel1) : (_vel1);
        float _dphi4 = this->reverse ? (-_vel4) : (_vel4);
        _tor1 = this->reverse ? (-_tor1) : (_tor1);
        _tor4 = this->reverse ? (-_tor4) : (_tor4);
        
        this->vmc.Resolve(_phi1, _phi4);

        /* xdot */
        float qdot[2] = {_dphi1, _dphi4};
        float xdot[2] = {0.0f, 0.0f};
        this->vmc.VMCVelCal(qdot, xdot);

        /* len params */
        this->len = this->vmc.GetLen();
        this->dlen = xdot[0];

        /* alpha params */
        float phi = this->vmc.GetPhi();
        this->alpha = Numeric::LoopFloatConstrain(phi-0.5f*PI+_pitch, -PI, PI);
        this->dalpha = xdot[1] + _dpitch;

        /* inverse dynamics */
        float Treal[2] = {_tor1, _tor4};
        float Trev[2] = {0.0f, 0.0f};
        this->vmc.VMCRevCal(Trev, Treal);
        float P = Trev[0]*arm_cos_f32(this->alpha) + Trev[1]/this->len*arm_sin_f32(this->alpha);
        float ddlen = (this->dlen-this->prev_dlen)*1000.0f;
        this->N = P + WHEEL_MASS*(_az - ddlen*arm_cos_f32(this->alpha)) + F_SPRING*arm_cos_f32(_phi1-phi);

        /* neutral and flat detection */
        if (Numeric::abs(this->alpha) < 0.25f)
            this->neutral_count++;
        else
            this->neutral_count = 0;
        this->neutral = this->neutral_count > 200;
        this->flat = (2.5f <= phi && phi <= 3.1f);

        this->prev_dlen = this->dlen;
    }

    void ForwardDynamics(float *_F, float *_T)
    {
        this->vmc.VMCCal(_F, _T);
    }

};