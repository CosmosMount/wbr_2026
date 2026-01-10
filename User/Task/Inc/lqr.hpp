# pragma once
#include "math.hpp"
#include "config_chassis.hpp"

using namespace Numeric;

class LQR
{
protected:
    float LQRKcoeffs[40][6] =
    {
        /*Q = [20.00, 2.00, 40.00, 40.00, 400.00, 1.00, 400.00, 1.00, 3000.00, 20.00] R = [3.00, 3.00, 0.50, 0.50]*/
        /* a1 + a2*L_len + a3*R_len + a4*L_len^2 + a5*L_len*R_len + a6*R_len^2 */
        { 1.774951 , 3.822516, -3.595296, -6.940944, 4.216906, 2.541383}, 
        { 4.280360 , 2.426532, -11.100505, -11.331657, 20.069423, 6.920435}, 
        { 2.538043 , -2.876329, 1.900307, 3.413693, 0.323392, -2.190881}, 
        { 2.599119 , -3.099977, 2.108096, 3.608851, 0.391752, -2.485270}, 
        { 7.400492 , 41.249835, -15.602109, -34.800370, -13.715514, 19.056283}, 
        { 0.923310 , 5.865480, -2.775163, -1.177588, -1.516639, 3.145865}, 
        { 4.850994 , -13.254544, 26.648142, 11.718663, -1.118822, -27.137918}, 
        { 0.760807 , -1.618569, 2.737582, 0.284150, 2.206687, -1.523835}, 
        { 12.327740 , -27.247568, -17.876146, 25.129362, 24.693973, 9.341877}, 
        { 1.870799 , -3.479440, -3.430869, 2.253180, 4.952602, 2.239806}, 
        { 1.774951 , -3.595296, 3.822516, 2.541383, 4.216906, -6.940944}, 
        { 4.280360 , -11.100505, 2.426532, 6.920435, 20.069423, -11.331657}, 
        { -2.538043 , -1.900307, 2.876329, 2.190881, -0.323392, -3.413693}, 
        { -2.599119 , -2.108096, 3.099977, 2.485270, -0.391752, -3.608851}, 
        { 4.850994 , 26.648142, -13.254544, -27.137918, -1.118822, 11.718663}, 
        { 0.760807 , 2.737582, -1.618569, -1.523835, 2.206687, 0.284150}, 
        { 7.400492 , -15.602109, 41.249835, 19.056283, -13.715514, -34.800370}, 
        { 0.923310 , -2.775163, 5.865480, 3.145865, -1.516639, -1.177588}, 
        { 12.327740 , -17.876146, -27.247568, 9.341877, 24.693973, 25.129362}, 
        { 1.870799 , -3.430869, -3.479440, 2.239806, 4.952602, 2.253180}, 
        { -0.568461 , -14.483698, 14.005462, 24.807893, -2.655796, -18.036708}, 
        { -1.318330 , -25.222785, 27.352363, 45.064096, -8.436347, -35.315258}, 
        { 1.369912 , 5.026716, 1.946850, -9.644171, 1.529556, -3.401726}, 
        { 1.418034 , 5.248822, 2.023417, -9.807891, 1.749366, -3.347337}, 
        { -19.951119 , 5.671331, -10.863205, -22.663590, -42.507683, 17.675760}, 
        { -1.714783 , -2.437055, 0.585271, -2.998339, -2.979317, -1.556313}, 
        { 7.196209 , 23.397074, 16.663780, -30.136670, 44.269582, -4.896498}, 
        { 0.717815 , 0.896360, 2.446913, 0.988471, 2.775512, 3.480709}, 
        { 51.265593 , 62.127905, -33.185676, -79.958550, -10.911212, 45.032194}, 
        { 4.787161 , 6.160856, -2.656650, -6.018664, -2.033780, 2.820995}, 
        { -0.568461 , 14.005462, -14.483698, -18.036708, -2.655796, 24.807893}, 
        { -1.318330 , 27.352363, -25.222785, -35.315258, -8.436347, 45.064096}, 
        { -1.369912 , -1.946850, -5.026716, 3.401726, -1.529556, 9.644171}, 
        { -1.418034 , -2.023417, -5.248822, 3.347337, -1.749366, 9.807891}, 
        { 7.196209 , 16.663780, 23.397074, -4.896498, 44.269582, -30.136670}, 
        { 0.717815 , 2.446913, 0.896360, 3.480709, 2.775512, 0.988471}, 
        { -19.951119 , -10.863205, 5.671331, 17.675760, -42.507683, -22.663590}, 
        { -1.714783 , 0.585271, -2.437055, -1.556313, -2.979317, -2.998339}, 
        { 51.265593 , -33.185676, 62.127905, 45.032194, -10.911212, -79.958550}, 
        { 4.787161 , -2.656650, 6.160856, 2.820995, -2.033780, -6.018664}
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
