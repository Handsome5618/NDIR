#include "coefficient.h"

/**
 * @brief    标定系数计算
 * @param  	 坐标
 * @param    坐标点数
 * @param    斜率
 * @param    截距
 * @retval   系数计算状态值
 */
int LineInfo(POINT_2 *pt_input, int ptNumbers, double *k, double *b)
{
    int nRet = 0;

    double X_aver = 0;
    double Y_aver = 0;
    double A      = 0;
    double B      = 0;

    double _2_XY_av = 0;
    double _2_XX_av = 0;

    do {
        if (ptNumbers < 2) {
            nRet = -1;
            break;
        }

        for (int loop = 0; loop < ptNumbers; loop++) {
            X_aver += pt_input[loop].x;
            Y_aver += pt_input[loop].y;

            _2_XY_av += pt_input[loop].x * pt_input[loop].y;
            _2_XX_av += pt_input[loop].x * pt_input[loop].x;
        }
        X_aver /= ptNumbers;
        Y_aver /= ptNumbers;

        _2_XY_av /= ptNumbers;
        _2_XX_av /= ptNumbers;

        for (int loop = 0; loop < ptNumbers; loop++) {
            A += fabs((pt_input[loop].x - X_aver) * (pt_input[loop].y - Y_aver));
            B += fabs((pt_input[loop].x - X_aver) * (pt_input[loop].x - X_aver));
        }

        if (B < 0.000001) {
            nRet = -2;
            break;
        }
        *k = (_2_XY_av - X_aver * Y_aver) / (_2_XX_av - X_aver * X_aver);
        *b = Y_aver - (*k) * X_aver;
    } while (0);

    return nRet;
}
