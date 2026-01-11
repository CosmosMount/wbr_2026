# pragma once
#include "math.hpp"
#include "config_chassis.hpp"

using namespace Numeric;

class LQR
{
protected:
    float LQRKcoeffs[40][6] =
    {
        /*Q = [60.00, 60.00, 60.00, 60.00, 200.00, 1.00, 200.00, 1.00, 2000.00, 20.00] R = [3.00, 3.00, 0.50, 0.50]*/
        /* a1 + a2*L_len + a3*R_len + a4*L_len^2 + a5*L_len*R_len + a6*R_len^2 */
        { 3.053779 , 6.862396, -6.378647, -12.146729, 5.157265, 6.518338}, 
        { 5.918868 , 7.925038, -13.912416, -17.273115, 16.570189, 13.577152}, 
        { 3.118708 , -3.677383, 2.575014, 4.172620, 0.677736, -2.977237}, 
        { 3.180426 , -3.916019, 2.787997, 4.354634, 0.754468, -3.251126}, 
        { 5.664331 , 50.416868, -17.700557, -26.138286, -28.128212, 24.936633}, 
        { 0.926182 , 7.764548, -3.378066, 0.759578, -3.774198, 4.646294}, 
        { 3.978390 , -8.946567, 33.147319, 1.679606, 0.854196, -25.560550}, 
        { 0.782395 , -1.070640, 3.797836, -1.080452, 3.309067, -0.392776}, 
        { 11.299559 , -19.717575, -16.963568, 14.393889, 23.818115, 7.853629}, 
        { 2.007679 , -2.961134, -3.798239, 1.337771, 5.067088, 2.585769}, 
        { 3.053779 , -6.378647, 6.862396, 6.518338, 5.157265, -12.146729}, 
        { 5.918868 , -13.912416, 7.925038, 13.577152, 16.570189, -17.273115}, 
        { -3.118708 , -2.575014, 3.677383, 2.977237, -0.677736, -4.172620}, 
        { -3.180426 , -2.787997, 3.916019, 3.251126, -0.754468, -4.354634}, 
        { 3.978390 , 33.147319, -8.946567, -25.560550, 0.854196, 1.679606}, 
        { 0.782395 , 3.797836, -1.070640, -0.392776, 3.309067, -1.080452}, 
        { 5.664331 , -17.700557, 50.416868, 24.936633, -28.128212, -26.138286}, 
        { 0.926182 , -3.378066, 7.764548, 4.646294, -3.774198, 0.759578}, 
        { 11.299559 , -16.963568, -19.717575, 7.853629, 23.818115, 14.393889}, 
        { 2.007679 , -3.798239, -2.961134, 2.585769, 5.067088, 1.337771}, 
        { -1.185468 , -26.025001, 25.040232, 50.479534, -4.512698, -38.048342}, 
        { -2.176777 , -42.124078, 42.884734, 81.345842, -9.937830, -63.730468}, 
        { 1.540774 , 6.459420, 1.972329, -12.558203, 1.324664, -3.456318}, 
        { 1.587218 , 6.724736, 2.065715, -12.804926, 1.623297, -3.462358}, 
        { -13.959001 , -27.433486, 5.716750, 13.333251, -24.086333, -17.521525}, 
        { -1.580486 , -5.968394, 3.298993, 0.524463, 0.743994, -7.711276}, 
        { 5.303857 , 4.031428, 22.914135, 13.176647, 32.067924, -8.782712}, 
        { 0.671350 , -2.108476, 3.406389, 8.448368, -0.220208, 2.284411}, 
        { 41.850734 , 38.414550, -16.187582, -39.338060, -11.600082, 18.836643}, 
        { 4.557204 , 3.809360, -0.440881, -1.433904, -2.419580, -0.707162}, 
        { -1.185468 , 25.040232, -26.025001, -38.048342, -4.512698, 50.479534}, 
        { -2.176777 , 42.884734, -42.124078, -63.730468, -9.937830, 81.345842}, 
        { -1.540774 , -1.972329, -6.459420, 3.456318, -1.324664, 12.558203}, 
        { -1.587218 , -2.065715, -6.724736, 3.462358, -1.623297, 12.804926}, 
        { 5.303857 , 22.914135, 4.031428, -8.782712, 32.067924, 13.176647}, 
        { 0.671350 , 3.406389, -2.108476, 2.284411, -0.220208, 8.448368}, 
        { -13.959001 , 5.716750, -27.433486, -17.521525, -24.086333, 13.333251}, 
        { -1.580486 , 3.298993, -5.968394, -7.711276, 0.743994, 0.524463}, 
        { 41.850734 , -16.187582, 38.414550, 18.836643, -11.600082, -39.338060}, 
        { 4.557204 , -0.440881, 3.809360, -0.707162, -2.419580, -1.433904} 
    };

    float LQRKBuf[40] = {0};
    float LQROutBuf[4] = {0};
    float LQRXerrorBuf[10] = {0};

    float * LQRXRefX;
    float * LQRXObsX;

public:

    /* [Tl;Tr;Tpl;Tpr] = -K(X_obs - X_ref) */
    void LQRCal(float *Tout)
    {
        // 1. Calculate Error: X_err = X_obs - X_ref
        float err[10] = {0};
        for (int i=0; i<10; i++)
        {
            err[i] = this->LQRXObsX[i] - this->LQRXRefX[i];
        }    
        
        // 2. Calculate U = -K * X_err
        // Matrix multiplication: [4x10] * [10x1] = [4x1]
        for (int i=0; i<4; i++)
        {
            float temp = 0.0f;
            for (int j=0; j<10; j++)
            {
                temp += LQRKBuf[i*10+j]*err[j];
            }   
            Tout[i] = temp;
        }
            
    }

    /* 根据腿长更新使用的矩阵k */
    void refreshLQRK(float L_LegLenth, float R_LegLenth, bool ifOffground)
    {
        L_LegLenth = (L_LegLenth < LQR_MIN_LEN_CTRL) ? LQR_MIN_LEN_CTRL : L_LegLenth;
        L_LegLenth = (L_LegLenth > LQR_MAX_LEN_CTRL) ? LQR_MAX_LEN_CTRL : L_LegLenth;
        R_LegLenth = (R_LegLenth < LQR_MIN_LEN_CTRL) ? LQR_MIN_LEN_CTRL : R_LegLenth;
        R_LegLenth = (R_LegLenth > LQR_MAX_LEN_CTRL) ? LQR_MAX_LEN_CTRL : R_LegLenth;
        //保留两位小数
        L_LegLenth = roundf(L_LegLenth * 100) / 100.0f;
        R_LegLenth = roundf(R_LegLenth * 100) / 100.0f;

        /* a1 + a2*L_len + a3*R_len + a4*L_len^2 + a5*L_len*R_len + a6*R_len^2 */ 
        if (!ifOffground)
        {
            for(int i = 0; i < 40; i++)
            {
                LQRKBuf[i] = LQRKcoeffs[i][0] + LQRKcoeffs[i][1] * L_LegLenth + LQRKcoeffs[i][2] * R_LegLenth + LQRKcoeffs[i][3] * L_LegLenth * L_LegLenth + LQRKcoeffs[i][4] * L_LegLenth * R_LegLenth + LQRKcoeffs[i][5] * R_LegLenth * R_LegLenth;
            }
        }
        else
        {
            for(int i = 0; i < 40; i++)
            {
                LQRKBuf[i] = 0.0f;
            }
            for(int j = 24; j < 28; j++)
            {
                LQRKBuf[j] = LQRKcoeffs[j][0] + LQRKcoeffs[j][1] * L_LegLenth + LQRKcoeffs[j][2] * R_LegLenth + LQRKcoeffs[j][3] * L_LegLenth * L_LegLenth + LQRKcoeffs[j][4] * L_LegLenth * R_LegLenth + LQRKcoeffs[j][5] * R_LegLenth * R_LegLenth;
            }
            for(int k = 34; k < 38; k++)
            {
                LQRKBuf[k] = LQRKcoeffs[k][0] + LQRKcoeffs[k][1] * L_LegLenth + LQRKcoeffs[k][2] * R_LegLenth + LQRKcoeffs[k][3] * L_LegLenth * L_LegLenth + LQRKcoeffs[k][4] * L_LegLenth * R_LegLenth + LQRKcoeffs[k][5] * R_LegLenth * R_LegLenth;
            }
        }
    }

    // Modified to accept raw float pointers or extract data pointer from arm_matrix_instance_f32 if needed
    void InitMatX(float *pMatXRef, float *pMatXObs) 
    {
        // Store pointers to the actual data arrays
        this->LQRXRefX = pMatXRef;
        this->LQRXObsX = pMatXObs;
    }

};
