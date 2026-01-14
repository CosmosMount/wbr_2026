# pragma once
#include "math.hpp"
#include "config_chassis.hpp"

using namespace Numeric;

class LQR
{
protected:
    float LQRKcoeffs[40][6] =
    {
        /*Q = [80.00, 80.00, 60.00, 60.00, 1000.00, 20.00, 1000.00, 20.00, 2000.00, 10.00] R = [5.00, 5.00, 0.50, 0.50]*/
        /* a1 + a2*L_len + a3*R_len + a4*L_len^2 + a5*L_len*R_len + a6*R_len^2 */
        { 2.693422 , 6.007121, -5.882094, -9.937743, 5.957879, 4.774820}, 
        { 6.332054 , 6.940946, -15.927075, -18.468410, 25.510623, 10.913127}, 
        { 2.415759 , -2.850349, 1.804163, 3.068359, 0.428135, -1.936464}, 
        { 2.475957 , -3.126298, 2.077923, 3.249273, 0.621665, -2.338965}, 
        { 7.776956 , 62.012334, -19.375855, -50.559504, -25.320802, 24.878092}, 
        { 1.422734 , 9.094091, -3.566670, -2.614218, -3.136400, 4.215581}, 
        { 4.705450 , -12.036692, 38.135629, 4.042318, 6.652413, -39.165894}, 
        { 0.979702 , -1.734086, 4.329311, -0.542927, 4.738630, -2.568092}, 
        { 14.347928 , -25.553618, -20.621124, 13.461324, 32.260459, 9.122424}, 
        { 2.161474 , -3.181235, -4.208259, 0.881501, 6.064920, 2.830380}, 
        { 2.693422 , -5.882094, 6.007121, 4.774820, 5.957879, -9.937743}, 
        { 6.332054 , -15.927075, 6.940946, 10.913127, 25.510623, -18.468410}, 
        { -2.415759 , -1.804163, 2.850349, 1.936464, -0.428135, -3.068359}, 
        { -2.475957 , -2.077923, 3.126298, 2.338965, -0.621665, -3.249273}, 
        { 4.705450 , 38.135629, -12.036692, -39.165894, 6.652413, 4.042318}, 
        { 0.979702 , 4.329311, -1.734086, -2.568092, 4.738630, -0.542927}, 
        { 7.776956 , -19.375855, 62.012334, 24.878092, -25.320802, -50.559504}, 
        { 1.422734 , -3.566670, 9.094091, 4.215581, -3.136400, -2.614218}, 
        { 14.347928 , -20.621124, -25.553618, 9.122424, 32.260459, 13.461324}, 
        { 2.161474 , -4.208259, -3.181235, 2.830380, 6.064920, 0.881501}, 
        { -2.016794 , -30.763794, 26.459244, 61.359652, -15.545633, -25.246416}, 
        { -4.936829 , -55.260799, 56.234106, 116.751917, -42.034322, -53.489448}, 
        { 1.628288 , 7.078339, 2.377340, -13.168159, 1.594732, -3.422372}, 
        { 1.680879 , 7.402036, 2.277312, -13.216640, 1.234379, -2.860900}, 
        { -40.095983 , 14.543034, 5.504171, -13.786809, -27.152836, -9.313030}, 
        { -6.097992 , 2.139504, 2.625267, -7.753949, 1.314876, -4.305317}, 
        { 3.701465 , 14.386697, 39.843273, 11.341766, 10.826798, -14.519761}, 
        { 0.714933 , -0.168842, 4.373762, 8.774425, -7.295594, 5.696788}, 
        { 30.621801 , 113.723247, -32.677356, -129.586100, -29.680748, 38.132169}, 
        { 2.495873 , 8.317196, -0.081144, -5.065323, -5.952782, -0.356339}, 
        { -2.016794 , 26.459244, -30.763794, -25.246416, -15.545633, 61.359652}, 
        { -4.936829 , 56.234106, -55.260799, -53.489448, -42.034322, 116.751917}, 
        { -1.628288 , -2.377340, -7.078339, 3.422372, -1.594732, 13.168159}, 
        { -1.680879 , -2.277312, -7.402036, 2.860900, -1.234379, 13.216640}, 
        { 3.701465 , 39.843273, 14.386697, -14.519761, 10.826798, 11.341766}, 
        { 0.714933 , 4.373762, -0.168842, 5.696788, -7.295594, 8.774425}, 
        { -40.095983 , 5.504171, 14.543034, -9.313030, -27.152836, -13.786809}, 
        { -6.097992 , 2.625267, 2.139504, -4.305317, 1.314876, -7.753949}, 
        { 30.621801 , -32.677356, 113.723247, 38.132169, -29.680748, -129.586100}, 
        { 2.495873 , -0.081144, 8.317196, -0.356339, -5.952782, -5.065323}
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
    void refreshLQRK(float L_LegLenth, float R_LegLenth, bool ifwheeloff)
    {
        L_LegLenth = (L_LegLenth < LQR_MIN_LEN_CTRL) ? LQR_MIN_LEN_CTRL : L_LegLenth;
        L_LegLenth = (L_LegLenth > LQR_MAX_LEN_CTRL) ? LQR_MAX_LEN_CTRL : L_LegLenth;
        R_LegLenth = (R_LegLenth < LQR_MIN_LEN_CTRL) ? LQR_MIN_LEN_CTRL : R_LegLenth;
        R_LegLenth = (R_LegLenth > LQR_MAX_LEN_CTRL) ? LQR_MAX_LEN_CTRL : R_LegLenth;
        //保留两位小数
        L_LegLenth = roundf(L_LegLenth * 100) / 100.0f;
        R_LegLenth = roundf(R_LegLenth * 100) / 100.0f;

        /* a1 + a2*L_len + a3*R_len + a4*L_len^2 + a5*L_len*R_len + a6*R_len^2 */ 
        if (!ifwheeloff)
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
