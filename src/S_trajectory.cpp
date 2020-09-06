#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "rt_type/tmwtypes.h"
#include "kinematics.h"
#include "math_tool.h"
#include "rt_type/rtwUtil.h"

/*
 * 功能--------------S曲线参数计算（S型速度规划，又称七段式轨迹）
 * 输入--------------q0:某一关节当前时刻角度（弧度值）
 *                   q1：某一关节需要到达的角度
 *                   limit[0]:最大速度
 *                   limit[1]：最大加速度
 *                   limit[2]：最大加加速度
 *                   limit[3]：最大速度比例，取值0~1，用以调节机械臂最大速度
 *                   workMode：工作模式，区分S曲线规划对象
 *                          workMode == 0:对角速度规划；
 *                          workMode == 1:对TCP线速度规划；
 *                          workMode == 2:对单个旋转角（beta、等效旋转角、moveC初末夹角）规划；
 *                   speedRate：机械臂最大速度比例，取值0~1，用于调节机械臂最大速度
 * 输出--------------para：规划出的S曲线的参数
 *                   para=[Ta, Tv, Td, Tj1, Tj2, q0, q1, v0, v1, vlim, amax,
 *                   amin, alima, alimd, jmax, jmin]
 */
void STrajectoryPara(double q0, double q1, double limit[4], double para[17])
{
    double vmax;
    double amax;
    double jmax;
    double sigma;
    double q_0;
    double q_1;
    double v_0;
    double v_1;
    double Tj1;
    double v_max;
    double a_max;
    double a_min;
    double j_max;
    double j_min;
    double Td;
    double d2;
    double Ta;
    double Tv_tmp;
    int exitg1;
    memset(&para[0], 0, sizeof(double) << 4);

    vmax = limit[0] * limit[3]; /* 最大速度为180°/s */ /* beta最大旋转角速度为1/s */ /* TCP最大线速度为1m/s,注意程序中以mm为单位 */
    amax = limit[1] * limit[3];                                                      /* 最大加速度为180°/s^2 */
    jmax = limit[2] * limit[3];                                                      /* 最大加加速度为180°/s^3 */

    /* 初末速度设置为0 */
    /*  利用公式（3.31）（3.32）转化得到实际的q_0、q_1、v_max、a_max */
    sigma = q1 - q0;

    if (sigma == 0.0)
    {
        para[5] = q0;
        para[6] = q1;
        para[16] = 1.0;
        return;
    }

    b_sign(&sigma);
    para[16] = sigma;
    q_0 = sigma * q0;
    q_1 = sigma * q1;
    v_0 = sigma * 0.0;
    v_1 = sigma * 0.0;
    Tj1 = (sigma + 1.0) / 2.0;
    sigma = (sigma - 1.0) / 2.0;
    v_max = Tj1 * vmax + sigma * -vmax;
    a_max = Tj1 * amax + sigma * -amax;
    a_min = Tj1 * -amax + sigma * amax;
    j_max = Tj1 * jmax + sigma * -jmax;
    j_min = Tj1 * -jmax + sigma * jmax;

    /*  判断是否达到最大速度 */
    Td = v_max - v_0;
    d2 = a_max * a_max;
    if (Td * j_max < d2)
    {
        Tj1 = Td / j_max;
        b_sqrt(&Tj1);

        /*  达不到a_max */
        Ta = 2.0 * Tj1;
        sigma = j_max * Tj1;
    }
    else
    {
        Tj1 = a_max / j_max;

        /*  能够达到a_max */
        Ta = Tj1 + Td / a_max;
        sigma = a_max;
    }

    if ((v_max - v_1) * j_max < d2)
    {
        vmax = (v_max - v_1) / j_max;
        b_sqrt(&vmax);

        /*  达不到a_min */
        Td = 2.0 * vmax;
        amax = -j_max * vmax;
    }
    else
    {
        vmax = a_max / j_max;

        /*  能够达到a_min */
        Td = vmax + (v_max - v_1) / a_max;
        amax = -a_max;
    }

    /*  根据（3.25）计算匀速段时间 */
    Tv_tmp = q_1 - q_0;
    jmax = (Tv_tmp / v_max - Ta / 2.0 * (1.0 + v_0 / v_max)) - Td / 2.0 * (1.0 + v_1 / v_max);

    /*  对Tv进行讨论 */
    if (jmax > 0.0)
    {
        /*  达到最大速度v_max，即存在匀速阶段 */
        para[0] = Ta;
        para[1] = jmax;
        para[2] = Td;
        para[3] = Tj1;
        para[4] = vmax;
        para[5] = q_0;
        para[6] = q_1;
        para[7] = v_0;
        para[8] = v_1;
        para[9] = v_max;
        para[10] = a_max;
        para[11] = a_min;
        para[12] = sigma;
        para[13] = amax;
        para[14] = j_max;
        para[15] = j_min;
    }
    else
    {
        /*  达不到最大速度，即匀速阶段Tv=0 */
        /*  假设最大加速度和最小加速度均能达到 */
        vmax = a_max / j_max;
        amax = j_max * j_max;
        jmax = 2.0 * (v_0 * v_0 + v_1 * v_1);
        Tj1 = v_0 + v_1;
        v_max = 4.0 * Tv_tmp;
        sigma = (rt_powd_snf(a_max, 4.0) / amax + jmax) + a_max * (v_max - 2.0 *
                                                                               (a_max / j_max) * Tj1);
        Td = sigma;
        b_sqrt(&Td);
        Ta = ((d2 / j_max - 2.0 * v_0) + Td) / (2.0 * a_max);
        b_sqrt(&sigma);
        Td = ((a_max * a_max / j_max - 2.0 * v_1) + sigma) / (2.0 * a_max);

        /*  对Ta和Td进行讨论 */
        if ((Ta < 0.0) || (Td < 0.0))
        {
            if (Ta < 0.0)
            {
                /*  没有加速段，只有减速段 */
                sigma = v_1 + v_0;
                Td = j_max * (j_max * (Tv_tmp * Tv_tmp) + sigma * sigma * (v_1 - v_0));
                b_sqrt(&Td);
                vmax = (j_max * (q_1 - q_0) - Td) / (j_max * (v_1 + v_0));
                para[0] = 0.0;
                para[1] = 0.0;
                para[2] = 2.0 * (q_1 - q_0) / (v_0 + v_1);
                para[3] = 0.0;
                para[4] = vmax;
                para[5] = q_0;
                para[6] = q_1;
                para[7] = v_0;
                para[8] = v_1;
                para[9] = 0.0;
                para[10] = a_max;
                para[11] = a_min;
                para[12] = 0.0;
                para[13] = -j_max * vmax;
                para[14] = j_max;
                para[15] = j_min;
            }
            else
            {
                if (Td < 0.0)
                {
                    /*  没有减速段，只有加速段 */
                    Td = 2.0 * (q_1 - q_0) / (v_0 + v_1);
                    sigma = v_1 + v_0;
                    Tj1 = (j_max * (q_1 - q_0) - sqrt(j_max * (j_max * (Tv_tmp * Tv_tmp)) - sigma * sigma * (v_1 - v_0))) / (j_max * (v_1 + v_0));
                    d2 = j_max * Tj1;
                    para[0] = Td;
                    para[1] = 0.0;
                    para[2] = 0.0;
                    para[3] = Tj1;
                    para[4] = 0.0;
                    para[5] = q_0;
                    para[6] = q_1;
                    para[7] = v_0;
                    para[8] = v_1;
                    para[9] = v_0 + d2 * (Td - Tj1);
                    para[10] = a_max;
                    para[11] = a_min;
                    para[12] = d2;
                    para[13] = 0.0;
                    para[14] = j_max;
                    para[15] = j_min;
                }
            }
        }
        else if ((Ta >= 2.0 * vmax) && (Td >= 2.0 * vmax))
        {
            /*  加速段和减速段都能达到最大加速度 */
            para[0] = Ta;
            para[1] = 0.0;
            para[2] = Td;
            para[3] = vmax;
            para[4] = vmax;
            para[5] = q_0;
            para[6] = q_1;
            para[7] = v_0;
            para[8] = v_1;
            para[9] = a_max * (Ta - vmax);
            para[10] = a_max;
            para[11] = a_min;
            para[12] = a_max;
            para[13] = -a_max;
            para[14] = j_max;
            para[15] = j_min;
        }
        else
        {
            /*  加速和减速阶段至少有一段不能达到最大加速度 */
            /*  系统取0<lambda<1 */
            do
            {
                exitg1 = 0;
                if ((Ta < 2.0 * vmax) || (Td < 2.0 * vmax))
                {
                    /*  循环 */
                    a_max *= 0.99;
                    vmax = a_max / j_max;
                    sigma = (rt_powd_snf(a_max, 4.0) / amax + jmax) + a_max * (v_max - 2.0 * vmax * Tj1);
                    Ta = ((a_max * a_max / j_max - 2.0 * v_0) + sqrt(sigma)) / (2.0 * a_max);
                    Td = ((a_max * a_max / j_max - 2.0 * v_1) + sqrt(sigma)) / (2.0 * a_max);
                    if ((Ta < 0.0) || (Td < 0.0))
                    {
                        if (Ta < 0.0)
                        {
                            /*  没有加速段，只有减速段 */
                            sigma = v_1 + v_0;
                            vmax = (j_max * (q_1 - q_0) - sqrt(j_max * (j_max * (Tv_tmp * Tv_tmp) + sigma * sigma * (v_1 - v_0)))) / (j_max * (v_1 + v_0));
                            para[0] = 0.0;
                            para[1] = 0.0;
                            para[2] = 2.0 * (q_1 - q_0) / (v_0 + v_1);
                            para[3] = 0.0;
                            para[4] = vmax;
                            para[5] = q_0;
                            para[6] = q_1;
                            para[7] = v_0;
                            para[8] = v_1;
                            para[9] = 0.0;
                            para[10] = a_max;
                            para[11] = a_min;
                            para[12] = 0.0;
                            para[13] = -j_max * vmax;
                            para[14] = j_max;
                            para[15] = j_min;
                            exitg1 = 1;
                        }
                        else
                        {
                            if (Td < 0.0)
                            {
                                /*  没有减速段，只有加速段 */
                                vmax = 2.0 * Tv_tmp / Tj1;
                                sigma = v_1 + v_0;
                                Tj1 = (j_max * Tv_tmp - sqrt(j_max * (j_max * (Tv_tmp * Tv_tmp)) - sigma * sigma * (v_1 - v_0))) / (j_max * (v_1 + v_0));
                                sigma = j_max * Tj1;
                                para[0] = vmax;
                                para[1] = 0.0;
                                para[2] = 0.0;
                                para[3] = Tj1;
                                para[4] = 0.0;
                                para[5] = q_0;
                                para[6] = q_1;
                                para[7] = v_0;
                                para[8] = v_1;
                                para[9] = v_0 + sigma * (vmax - Tj1);
                                para[10] = a_max;
                                para[11] = a_min;
                                para[12] = sigma;
                                para[13] = 0.0;
                                para[14] = j_max;
                                para[15] = j_min;
                                exitg1 = 1;
                            }
                        }
                    }
                    else
                    {
                        if ((Ta >= 2.0 * vmax) && (Td >= 2.0 * vmax))
                        {
                            /*  加速段和减速段都能达到最大加速度 */
                            para[0] = Ta;
                            para[1] = 0.0;
                            para[2] = Td;
                            para[3] = vmax;
                            para[4] = vmax;
                            para[5] = q_0;
                            para[6] = q_1;
                            para[7] = v_0;
                            para[8] = v_1;
                            para[9] = a_max * (Ta - vmax);
                            para[10] = a_max;
                            para[11] = a_min;
                            para[12] = a_max;
                            para[13] = -a_max;
                            para[14] = j_max;
                            para[15] = j_min;
                            exitg1 = 1;
                        }
                    }
                }
                else
                {
                    exitg1 = 1;
                }
            } while (exitg1 == 0);
        }
    }
}

/*
 * 功能：计算位移
 * 输入----------t：当前时间，从0开始
 *               para：由STrajectoryPara计算出的S曲线的16个参数
 * 输出----------double q：当前时刻位置值
 */
double S_position(double t, const double para[17])
{
    double q;
    double T_tmp;
    double T;
    q = 0.0;
    T_tmp = para[0] + para[1];
    T = T_tmp + para[2];

    if (T == 0.0)
    {
        return para[16] * para[5];
    }

    /*  加速段 */
    if ((t >= 0.0) && (t < para[3]))
    {
        q = (para[5] + para[7] * t) + para[14] * rt_powd_snf(t, 3.0) / 6.0;
    }
    else if ((t >= para[3]) && (t < para[0] - para[3]))
    {
        q = (para[5] + para[7] * t) + para[12] / 6.0 * ((3.0 * (t * t) - 3.0 * para[3] * t) + para[3] * para[3]);
    }
    else if ((t >= para[0] - para[3]) && (t < para[0]))
    {
        T_tmp = para[0] - t;
        q = ((para[5] + (para[9] + para[7]) * (para[0] / 2.0)) - para[9] * T_tmp) -
            para[15] * (rt_powd_snf(T_tmp, 3.0) / 6.0);

        /*  匀速段 */
    }
    else if ((t >= para[0]) && (t < T_tmp))
    {
        q = (para[5] + (para[9] + para[7]) * (para[0] / 2.0)) + para[9] * (t - para[0]);

        /*  减速段 */
    }
    else if ((t >= T_tmp) && (t < (T - para[2]) + para[4]))
    {
        q = ((para[6] - (para[9] + para[8]) * (para[2] / 2.0)) + para[9] * ((t - T) + para[2])) - para[14] * (rt_powd_snf((t - T) + para[2], 3.0) / 6.0);
    }
    else if ((t >= (T - para[2]) + para[4]) && (t < T - para[4]))
    {
        T_tmp = (t - T) + para[2];
        q = ((para[6] - (para[9] + para[8]) * (para[2] / 2.0)) + para[9] * T_tmp) +
            para[13] / 6.0 * ((3.0 * (T_tmp * T_tmp) - 3.0 * para[4] * T_tmp) + para[4] * para[4]);
    }
    else
    {
        if ((t >= T - para[4]) && (t <= T))
        {
            T_tmp = T - t;
            q = (para[6] - para[8] * T_tmp) - para[14] * (rt_powd_snf(T_tmp, 3.0) / 6.0);
        }
    }

    if (t > para[0] + para[1] + para[2])
    {
        q = para[6];
    }

    return para[16] * q;
}
