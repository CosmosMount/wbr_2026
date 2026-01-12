# pragma once
#include "math.hpp"
#include "config_chassis.hpp"

using namespace Numeric;

class LQR
{
protected:
    float LQRKcoeffs[40][6] =
    {
        /*Q = [60.00, 80.00, 60.00, 60.00, 200.00, 0.50, 200.00, 0.50, 2000.00, 20.00] R = [5.00, 5.00, 0.50, 0.50]*/
        /* a1 + a2*L_len + a3*R_len + a4*L_len^2 + a5*L_len*R_len + a6*R_len^2 */
        { 2.364146 , 5.519353, -5.170510, -9.660678, 3.571859, 5.797114}, 
        { 4.847422 , 7.507903, -11.535924, -15.082479, 11.778272, 12.388152}, 
        { 2.425002 , -2.967792, 1.909820, 3.357915, 0.467618, -2.036412}, 
        { 2.486441 , -3.175175, 2.096210, 3.505728, 0.537953, -2.266944}, 
        { 4.506797 , 46.454242, -15.115518, -23.222464, -27.885463, 21.689053}, 
        { 0.757449 , 7.129959, -2.950242, 0.676640, -3.849981, 4.206181}, 
        { 3.213104 , -7.025845, 30.718049, -0.427055, -0.930301, -21.738492}, 
        { 0.662787 , -0.789085, 3.431326, -1.199962, 2.507296, -0.017228}, 
        { 9.283452 , -14.448076, -13.750145, 7.957664, 20.284605, 5.372485}, 
        { 1.722263 , -2.295137, -3.245937, 0.663585, 4.315168, 2.162771}, 
        { 2.364146 , -5.170510, 5.519353, 5.797114, 3.571859, -9.660678}, 
        { 4.847422 , -11.535924, 7.507903, 12.388152, 11.778272, -15.082479}, 
        { -2.425002 , -1.909820, 2.967792, 2.036412, -0.467618, -3.357915}, 
        { -2.486441 , -2.096210, 3.175175, 2.266944, -0.537953, -3.505728}, 
        { 3.213104 , 30.718049, -7.025845, -21.738492, -0.930301, -0.427055}, 
        { 0.662787 , 3.431326, -0.789085, -0.017228, 2.507296, -1.199962}, 
        { 4.506797 , -15.115518, 46.454242, 21.689053, -27.885463, -23.222464}, 
        { 0.757449 , -2.950242, 7.129959, 4.206181, -3.849981, 0.676640}, 
        { 9.283452 , -13.750145, -14.448076, 5.372485, 20.284605, 7.957664}, 
        { 1.722263 , -3.245937, -2.295137, 2.162771, 4.315168, 0.663585}, 
        { -1.161206 , -26.771810, 24.803094, 54.051207, -4.099808, -39.768594}, 
        { -2.243939 , -46.845856, 45.383798, 93.947927, -9.381132, -71.234981}, 
        { 1.503662 , 7.375759, 2.514461, -14.500376, 1.997978, -4.736857}, 
        { 1.550545 , 7.647048, 2.597291, -14.752641, 2.300515, -4.737501}, 
        { -13.840814 , -35.064158, 9.951531, 29.470051, -22.706796, -27.808353}, 
        { -1.407741 , -8.149706, 4.276505, 4.609261, 0.855247, -9.960743}, 
        { 5.648961 , -1.149137, 21.881016, 28.219270, 32.569551, -12.978379}, 
        { 0.645113 , -3.385868, 3.869774, 11.736844, -0.000046, 0.296167}, 
        { 41.805153 , 31.114477, -10.619105, -23.112742, -12.875817, 8.836074}, 
        { 4.521527 , 2.794249, 0.420571, 1.026300, -2.644514, -2.309553}, 
        { -1.161206 , 24.803094, -26.771810, -39.768594, -4.099808, 54.051207}, 
        { -2.243939 , 45.383798, -46.845856, -71.234981, -9.381132, 93.947927}, 
        { -1.503662 , -2.514461, -7.375759, 4.736857, -1.997978, 14.500376}, 
        { -1.550545 , -2.597291, -7.647048, 4.737501, -2.300515, 14.752641}, 
        { 5.648961 , 21.881016, -1.149137, -12.978379, 32.569551, 28.219270}, 
        { 0.645113 , 3.869774, -3.385868, 0.296167, -0.000046, 11.736844}, 
        { -13.840814 , 9.951531, -35.064158, -27.808353, -22.706796, 29.470051}, 
        { -1.407741 , 4.276505, -8.149706, -9.960743, 0.855247, 4.609261}, 
        { 41.805153 , -10.619105, 31.114477, 8.836074, -12.875817, -23.112742}, 
        { 4.521527 , 0.420571, 2.794249, -2.309553, -2.644514, 1.026300}
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
