/*
 * File: ThreePointCircle.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 19-Aug-2020 16:26:53
 */

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "rt_type/tmwtypes.h"
#include "math_tool.h"
#include "rt_type/rtwUtil.h"

/* Function Definitions */

bool ifWhileCond(const int x_size[2])
{
    return (x_size[0] != 0) && (x_size[1] != 0);
}

void singleOrHelper(const double a_data[], const int a_size[2], const double b_data[], bool y_data[], int y_size[2])
{
    int loop_ub;
    int i9;
    y_size[0] = a_size[0];
    y_size[1] = a_size[1];
    loop_ub = a_size[0] * a_size[1];
    for (i9 = 0; i9 < loop_ub; i9++)
    {
        y_data[i9] = ((a_data[i9] != 0.0) || (b_data[i9] != 0.0));
    }
}

void b_abs(const double x_data[], double y_data[], int y_size[2])
{
    y_size[0] = 1;
    y_size[1] = 3;
    y_data[0] = fabs(x_data[0]);
    y_data[1] = fabs(x_data[1]);
    y_data[2] = fabs(x_data[2]);
}

boolean_T all(const boolean_T x_data[])
{
    boolean_T y;
    int ix;
    boolean_T exitg1;
    y = true;
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix < 4))
    {
        if (!x_data[ix - 1])
        {
            y = false;
            exitg1 = true;
        }
        else
        {
            ix++;
        }
    }

    return y;
}

/*
 * TCP姿态可控/不变圆弧插补（笛卡尔空间）1
 * 功能--------根据三个空间点，计算出其圆心、半径、法向量、转动角度
 * 输入--------p1, p2, p3：三个空间点，1x3
 * 输出--------center：圆心1x3
 *             rad：半径
 *                 rad>0:   圆弧
 *                 rad = -1:输入数据有问题
 *                 rad = -2:三点共线
 *             normalVector:三点平面单位法向量，并将其作为等效旋转轴
 *             theta13:三点圆构成的二维平面的圆心与初末点构成的夹角，并将其作为等效旋转角
 *             T:三维空间到二维平面的转移矩阵
 *             rad = -1：给定点相同或者维度不一致
 *             rad = -2：给定点共线
 */
void ThreePointCircle(const double p1[3], const double p2[3], const double p3[3],
                      double center_data[], int center_size[2], double *rad,
                      double normalVector_data[], int normalVector_size[2],
                      double *theta13, double T_data[], int T_size[2])
{
    double v1_tmp;
    double v1[3];
    double v2[3];
    double b_v1_tmp;
    double c_v1_tmp;
    double cy;
    int iy;
    int jy;
    int tmp_size[2];
    int i8;
    double tmp_data[1];
    double bx;
    double ii_data[1];
    bool b_tmp_data[1];
    int b_tmp_size[2];
    double v1n[3];
    double w[3];
    double v[3];
    boolean_T b_normalVector_data[3];
    int ii_size[2];
    double b_a;
    double B;
    double A;
    double c_tmp_data[3];
    double b_w[16];
    double A_data[16];
    double b_B[4];
    signed char ipiv[4];
    int j;
    int b_b;
    int jj;
    int jp1j;
    int n;
    int ix;
    int ijA;
    center_size[0] = 1;
    center_size[1] = 1;
    center_data[0] = 0.0;
    normalVector_size[0] = 1;
    normalVector_size[1] = 1;
    normalVector_data[0] = 0.0;
    *theta13 = 0.0;
    T_size[0] = 1;
    T_size[1] = 1;
    T_data[0] = 0.0;

    /*  数据检查 */
    /*  检查数据输入格式是否正确 */
    /*  计算p1到p2的单位向量和p1到p3的单位向量 */
    /*  检查点是否相同 */
    v1_tmp = p2[0] - p1[0];
    v1[0] = v1_tmp;
    v2[0] = p3[0] - p1[0];
    b_v1_tmp = p2[1] - p1[1];
    v1[1] = b_v1_tmp;
    v2[1] = p3[1] - p1[1];
    c_v1_tmp = p2[2] - p1[2];
    v1[2] = c_v1_tmp;
    v2[2] = p3[2] - p1[2];
    cy = b_norm(v1);
    if (cy == 0.0)
    {
        iy = 1;
        jy = 1;
    }
    else
    {
        iy = 0;
        jy = 0;
    }

    tmp_size[0] = iy;
    tmp_size[1] = jy;
    iy *= jy;
    for (i8 = 0; i8 < iy; i8++)
    {
        tmp_data[0] = 1.0;
    }

    if (ifWhileCond(tmp_size))
    {
        /*      fprintf('输入点不能一样\n'); */
        *rad = -1.0;
    }
    else
    {
        bx = b_norm(v2);
        if (bx == 0.0)
        {
            iy = 1;
            jy = 1;
        }
        else
        {
            iy = 0;
            jy = 0;
        }

        iy *= jy;
        for (i8 = 0; i8 < iy; i8++)
        {
            ii_data[0] = 1.0;
        }

        singleOrHelper(tmp_data, tmp_size, ii_data, b_tmp_data, b_tmp_size);
        if (ifWhileCond(b_tmp_size))
        {
            /*      fprintf('输入点不能一样\n'); */
            *rad = -1.0;
        }
        else
        {
            /*  计算圆平面上的单位法向量 */
            /*  检查三点是否共线 */
            v1n[0] = v1_tmp / cy;
            w[0] = v2[0] / bx;
            v1n[1] = b_v1_tmp / cy;
            w[1] = v2[1] / bx;
            v1n[2] = c_v1_tmp / cy;
            w[2] = v2[2] / bx;
            cross(v1n, w, v);
            normalVector_size[0] = 1;
            normalVector_size[1] = 3;
            normalVector_data[0] = v[0];
            normalVector_data[1] = v[1];
            normalVector_data[2] = v[2];
            for (i8 = 0; i8 < 3; i8++)
            {
                b_normalVector_data[i8] = (normalVector_data[i8] == 0.0);
            }

            if (all(b_normalVector_data))
            {
                /*       fprintf('三个点共线\n'); */
                *rad = -2.0;
            }
            else
            {
                b_abs(normalVector_data, w, tmp_size);
                if (sum(w) < 1.0E-5)
                {
                    iy = 1;
                    jy = 1;
                }
                else
                {
                    iy = 0;
                    jy = 0;
                }

                ii_size[0] = iy;
                ii_size[1] = jy;
                if (ifWhileCond(ii_size))
                {
                    /*      fprintf('三点过于趋近直线\n'); */
                    *rad = -1.0;
                }
                else
                {
                    /*  计算新坐标系UVW轴 */
                    cross(v2, v1, v);
                    cy = b_norm(v);
                    bx = v[0] / cy;
                    w[0] = bx;
                    normalVector_data[0] = bx;
                    bx = v[1] / cy;
                    w[1] = bx;
                    normalVector_data[1] = bx;
                    bx = v[2] / cy;
                    w[2] = bx;
                    normalVector_data[2] = bx;
                    cross(w, v1n, v);
                    normalVector_size[0] = 1;
                    normalVector_size[1] = 3;

                    /*  计算投影 */
                    bx = dot(v1, v1n);
                    cy = dot(v2, v);

                    /*  计算圆心 */
                    b_a = dot(v2, v1n) - bx / 2.0;
                    B = bx / 2.0;
                    cy = ((b_a * b_a + cy * cy) - B * B) / (2.0 * cy);
                    center_size[0] = 1;
                    center_size[1] = 3;
                    bx /= 2.0;
                    center_data[0] = (p1[0] + bx * v1n[0]) + cy * v[0];
                    center_data[1] = (p1[1] + bx * v1n[1]) + cy * v[1];
                    center_data[2] = (p1[2] + bx * v1n[2]) + cy * v[2];

                    /*  半径 */
                    b_a = center_data[0] - p1[0];
                    B = center_data[1] - p1[1];
                    cy = center_data[2] - p1[2];
                    *rad = sqrt((b_a * b_a + B * B) + cy * cy);

                    /* 计算三维空间到二维平面的转移矩阵 */
                    cy = p3[2] - p2[2];
                    bx = p3[1] - p2[1];
                    A = b_v1_tmp * cy - c_v1_tmp * bx;
                    b_a = p3[0] - p2[0];
                    B = c_v1_tmp * b_a - v1_tmp * cy;
                    bx = v1_tmp * bx - b_v1_tmp * b_a;
                    cy = sqrt((A * A + B * B) + bx * bx);
                    v[0] = A / cy;
                    v[1] = B / cy;
                    v[2] = bx / cy;
                    w[0] = (p1[0] - center_data[0]) / *rad;
                    w[1] = (p1[1] - center_data[1]) / *rad;
                    w[2] = (p1[2] - center_data[2]) / *rad;
                    memcpy(&c_tmp_data[0], &center_data[0], (unsigned int)(3 * (int)sizeof(double)));
                    b_w[4] = v[1] * w[2] - v[2] * w[1];
                    b_w[5] = v[2] * w[0] - v[0] * w[2];
                    b_w[6] = v[0] * w[1] - v[1] * w[0];
                    b_w[0] = w[0];
                    b_w[8] = v[0];
                    b_w[12] = c_tmp_data[0];
                    b_w[1] = w[1];
                    b_w[9] = v[1];
                    b_w[13] = c_tmp_data[1];
                    b_w[2] = w[2];
                    b_w[10] = v[2];
                    b_w[14] = c_tmp_data[2];
                    b_w[3] = 0.0;
                    b_w[7] = 0.0;
                    b_w[11] = 0.0;
                    b_w[15] = 1.0;
                    T_size[0] = 4;
                    T_size[1] = 4;
                    memcpy(&T_data[0], &b_w[0], sizeof(double) << 4);

                    /*  求转换后的点 */
                    memcpy(&A_data[0], &T_data[0], (unsigned int)(16 * (int)sizeof(double)));
                    b_B[0] = p3[0];
                    ipiv[0] = 1;
                    b_B[1] = p3[1];
                    ipiv[1] = 2;
                    b_B[2] = p3[2];
                    ipiv[2] = 3;
                    b_B[3] = 1.0;
                    ipiv[3] = 4;
                    for (j = 0; j < 3; j++)
                    {
                        b_b = j * 5;
                        jj = j * 5;
                        jp1j = b_b + 2;
                        n = 4 - j;
                        jy = 0;
                        ix = b_b;
                        cy = fabs(A_data[b_b]);
                        for (iy = 2; iy <= n; iy++)
                        {
                            ix++;
                            bx = fabs(A_data[ix]);
                            if (bx > cy)
                            {
                                jy = iy - 1;
                                cy = bx;
                            }
                        }

                        if (A_data[jj + jy] != 0.0)
                        {
                            if (jy != 0)
                            {
                                iy = j + jy;
                                ipiv[j] = (signed char)(iy + 1);
                                cy = A_data[j];
                                A_data[j] = A_data[iy];
                                A_data[iy] = cy;
                                ix = j + 4;
                                iy += 4;
                                cy = A_data[ix];
                                A_data[ix] = A_data[iy];
                                A_data[iy] = cy;
                                ix += 4;
                                iy += 4;
                                cy = A_data[ix];
                                A_data[ix] = A_data[iy];
                                A_data[iy] = cy;
                                ix += 4;
                                iy += 4;
                                cy = A_data[ix];
                                A_data[ix] = A_data[iy];
                                A_data[iy] = cy;
                            }

                            i8 = (jj - j) + 4;
                            for (jy = jp1j; jy <= i8; jy++)
                            {
                                A_data[jy - 1] /= A_data[jj];
                            }
                        }

                        n = 2 - j;
                        jy = b_b + 4;
                        iy = jj;
                        for (b_b = 0; b_b <= n; b_b++)
                        {
                            cy = A_data[jy];
                            if (A_data[jy] != 0.0)
                            {
                                ix = jj;
                                i8 = iy + 6;
                                jp1j = (iy - j) + 8;
                                for (ijA = i8; ijA <= jp1j; ijA++)
                                {
                                    A_data[ijA - 1] += A_data[ix + 1] * -cy;
                                    ix++;
                                }
                            }

                            jy += 4;
                            iy += 4;
                        }

                        if (ipiv[j] != j + 1)
                        {
                            cy = b_B[j];
                            iy = ipiv[j] - 1;
                            b_B[j] = b_B[iy];
                            b_B[iy] = cy;
                        }
                    }

                    if (b_B[0] != 0.0)
                    {
                        for (jy = 2; jy < 5; jy++)
                        {
                            b_B[jy - 1] -= b_B[0] * A_data[jy - 1];
                        }
                    }

                    if (b_B[1] != 0.0)
                    {
                        for (jy = 3; jy < 5; jy++)
                        {
                            b_B[jy - 1] -= b_B[1] * A_data[jy + 3];
                        }
                    }

                    if (b_B[2] != 0.0)
                    {
                        for (jy = 4; jy < 5; jy++)
                        {
                            b_B[3] -= b_B[2] * A_data[11];
                        }
                    }

                    if (b_B[3] != 0.0)
                    {
                        b_B[3] /= A_data[15];
                        for (jy = 0; jy < 3; jy++)
                        {
                            b_B[jy] -= b_B[3] * A_data[jy + 12];
                        }
                    }

                    if (b_B[2] != 0.0)
                    {
                        b_B[2] /= A_data[10];
                        for (jy = 0; jy < 2; jy++)
                        {
                            b_B[jy] -= b_B[2] * A_data[jy + 8];
                        }
                    }

                    if (b_B[1] != 0.0)
                    {
                        b_B[1] /= A_data[5];
                        for (jy = 0; jy < 1; jy++)
                        {
                            b_B[0] -= b_B[1] * A_data[4];
                        }
                    }

                    if (b_B[0] != 0.0)
                    {
                        b_B[0] /= A_data[0];
                    }

                    /*  计算转动角度 */
                    if (b_B[1] < 0.0)
                    {
                        *theta13 = rt_atan2d_snf(b_B[1], b_B[0]) + 6.2831853071795862;
                    }
                    else
                    {
                        *theta13 = rt_atan2d_snf(b_B[1], b_B[0]);
                    }
                }
            }
        }
    }
}
