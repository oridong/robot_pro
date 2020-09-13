#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "rt_type/rtwUtil.h"
#include "math_tool.h"

/*
 * 旋转矩阵和四元数互换
 * 功能：等效旋转轴、及相应的旋转角[θ,rx,ry,yz]--转换成--旋转矩阵
 * 输入--------quaternionMatrix：四元数：等效旋转角、及相应的旋转轴[θ,rx,ry,yz]
 * 输出--------R：旋转矩阵3x3
 */
void quat2rot(const double quaternionMatrix_data[4], double R[9])
{
    double w;
    double z;
    double x;
    double y;
    double R_tmp;
    double b_R_tmp;
    double c_R_tmp;
    double d_R_tmp;
    double e_R_tmp;
    double f_R_tmp;
    w = cos(quaternionMatrix_data[0] / 2.0);
    z = sin(quaternionMatrix_data[0] / 2.0);
    x = quaternionMatrix_data[1] * z;
    y = quaternionMatrix_data[2] * z;
    z *= quaternionMatrix_data[3];
    R_tmp = 2.0 * (z * z);
    b_R_tmp = 2.0 * (y * y);
    R[0] = (1.0 - b_R_tmp) - R_tmp;
    c_R_tmp = x * y;
    d_R_tmp = z * w;
    R[3] = 2.0 * (c_R_tmp + d_R_tmp);
    e_R_tmp = x * z;
    f_R_tmp = y * w;
    R[6] = 2.0 * (e_R_tmp - f_R_tmp);
    R[1] = 2.0 * (c_R_tmp - d_R_tmp);
    c_R_tmp = 1.0 - 2.0 * (x * x);
    R[4] = c_R_tmp - R_tmp;
    R_tmp = y * z;
    d_R_tmp = x * w;
    R[7] = 2.0 * (R_tmp + d_R_tmp);
    R[2] = 2.0 * (e_R_tmp + f_R_tmp);
    R[5] = 2.0 * (R_tmp - d_R_tmp);
    R[8] = c_R_tmp - b_R_tmp;
}

/*
 * 功能：旋转矩阵--转换成--等效旋转轴、及相应的旋转角[θ,rx,ry,yz]
 * 输入：rotationMatrix-----相对旋转矩阵3x3
 * 改编自matlab工具箱函数
 * 姿态矩阵转换成四元数[w,x,y,z]
 * 然后再转换成等效旋转轴、及相应的旋转角
 * 算法本身限制，计算出来的θ都是正值
 */
void rot2quat(const double rotationMatrix[9], double rEquivalent[4])
{
    double trace;
    double q_idx_0;
    double q_idx_1;
    double q_idx_2;
    double q_idx_3;
    trace = (rotationMatrix[0] + rotationMatrix[4]) + rotationMatrix[8];

    if (trace > 0.0)
    {
        trace = sqrt(trace + 1.0);
        q_idx_0 = 0.5 * trace;
        q_idx_1 = (rotationMatrix[7] - rotationMatrix[5]) / (2.0 * trace);
        q_idx_2 = (rotationMatrix[2] - rotationMatrix[6]) / (2.0 * trace);
        q_idx_3 = (rotationMatrix[3] - rotationMatrix[1]) / (2.0 * trace);
    }
    else if ((rotationMatrix[4] > rotationMatrix[0]) && (rotationMatrix[4] > rotationMatrix[8]))
    {
        /*  max value at rotationMatrix(2,2) */
        trace = sqrt(((rotationMatrix[4] - rotationMatrix[0]) - rotationMatrix[8]) +
                     1.0);
        q_idx_2 = 0.5 * trace;
        if (trace != 0.0)
        {
            trace = 0.5 / trace;
        }

        q_idx_0 = (rotationMatrix[2] - rotationMatrix[6]) * trace;
        q_idx_1 = (rotationMatrix[3] + rotationMatrix[1]) * trace;
        q_idx_3 = (rotationMatrix[7] + rotationMatrix[5]) * trace;
    }
    else if (rotationMatrix[8] > rotationMatrix[0])
    {
        /*  max value at rotationMatrix(3,3) */
        trace = sqrt(((rotationMatrix[8] - rotationMatrix[0]) - rotationMatrix[4]) +
                     1.0);
        q_idx_3 = 0.5 * trace;
        if (trace != 0.0)
        {
            trace = 0.5 / trace;
        }

        q_idx_0 = (rotationMatrix[3] - rotationMatrix[1]) * trace;
        q_idx_1 = (rotationMatrix[2] + rotationMatrix[6]) * trace;
        q_idx_2 = (rotationMatrix[7] + rotationMatrix[5]) * trace;
    }
    else
    {
        /*  max value at rotationMatrix(1,1) */
        trace = sqrt(((rotationMatrix[0] - rotationMatrix[4]) - rotationMatrix[8]) +
                     1.0);
        q_idx_1 = 0.5 * trace;
        if (trace != 0.0)
        {
            trace = 0.5 / trace;
        }

        q_idx_0 = (rotationMatrix[7] - rotationMatrix[5]) * trace;
        q_idx_2 = (rotationMatrix[3] + rotationMatrix[1]) * trace;
        q_idx_3 = (rotationMatrix[2] + rotationMatrix[6]) * trace;
    }

    trace = 2 * acos(q_idx_0);
    rEquivalent[0] = trace;
    if (trace == 0.0)
    {
        rEquivalent[1] = 0.0;
        rEquivalent[2] = 0.0;
        rEquivalent[3] = 0.0;
    }
    else
    {
        trace = sin(trace / 2.0);
        rEquivalent[1] = q_idx_1 / trace;
        rEquivalent[2] = q_idx_2 / trace;
        rEquivalent[3] = q_idx_3 / trace;
    }
}

/*
 * 功能：求出绕某向量n旋转angle角度的旋转矩阵
 * 输入-----------n：向量1x3，可以是非单位向量
 *               angle:旋转角度
 * 输出----------R：旋转矩阵3x3
 */
void VectorRot(double n[3], double angle, double R[9])
{
    double R_tmp;
    double b_R_tmp;
    double c_R_tmp;
    double d_R_tmp;
    double e_R_tmp;
    double f_R_tmp;
    R_tmp = b_norm(n);
    n[0] /= R_tmp;
    n[1] /= R_tmp;
    n[2] /= R_tmp;

    /* 单位化 */
    R_tmp = cos(angle);
    R[0] = n[0] * n[0] * (1.0 - R_tmp) + R_tmp;
    b_R_tmp = sin(angle);
    c_R_tmp = n[0] * n[1] * (1.0 - cos(angle));
    d_R_tmp = n[2] * b_R_tmp;
    R[3] = c_R_tmp - d_R_tmp;
    e_R_tmp = n[0] * n[2] * (1.0 - cos(angle));
    f_R_tmp = n[1] * b_R_tmp;
    R[6] = e_R_tmp + f_R_tmp;
    R[1] = c_R_tmp + d_R_tmp;
    R[4] = n[1] * n[1] * (1.0 - cos(angle)) + R_tmp;
    c_R_tmp = n[1] * n[2] * (1.0 - cos(angle));
    b_R_tmp *= n[0];
    R[7] = c_R_tmp - b_R_tmp;
    R[2] = e_R_tmp - f_R_tmp;
    R[5] = c_R_tmp + b_R_tmp;
    R[8] = n[2] * n[2] * (1.0 - cos(angle)) + R_tmp;
}

/*
 * 正运动学
 * 功能----------依据MDH四参数，计算相邻坐标系转移矩阵
 * 输入----------MDH四参数
 * 输出----------相邻坐标系转移矩阵4x4
 */
void CoordinateTrans(double alpha0, double a0, double theta1, double d1, double coordinateTrans[16])
{
    double coordinateTrans_tmp;
    double b_coordinateTrans_tmp;
    double c_coordinateTrans_tmp;
    double coordinateTrans_tmp_tmp;
    coordinateTrans_tmp = cos(theta1);
    b_coordinateTrans_tmp = sin(theta1);
    c_coordinateTrans_tmp = cos(alpha0);
    coordinateTrans_tmp_tmp = sin(alpha0);

    coordinateTrans[0] = coordinateTrans_tmp;
    coordinateTrans[4] = -b_coordinateTrans_tmp;
    coordinateTrans[8] = 0.0;
    coordinateTrans[12] = a0;
    coordinateTrans[1] = b_coordinateTrans_tmp * c_coordinateTrans_tmp;
    coordinateTrans[5] = coordinateTrans_tmp * c_coordinateTrans_tmp;
    coordinateTrans[9] = -coordinateTrans_tmp_tmp;
    coordinateTrans[13] = -coordinateTrans_tmp_tmp * d1;
    coordinateTrans[2] = b_coordinateTrans_tmp * coordinateTrans_tmp_tmp;
    coordinateTrans[6] = coordinateTrans_tmp * coordinateTrans_tmp_tmp;
    coordinateTrans[10] = c_coordinateTrans_tmp;
    coordinateTrans[14] = c_coordinateTrans_tmp * d1;
    coordinateTrans[3] = 0.0;
    coordinateTrans[7] = 0.0;
    coordinateTrans[11] = 0.0;
    coordinateTrans[15] = 1.0;
}

/*
 * 正运动学2
 * 功能----------根据7个关节角计算末端位姿
 * 输入----------七个关节角1x7
 * 输出----------末端位姿4x4
 */
void ForwardKinematics(const double angle[7], double T07[16])
{
    double T01[16];
    double T12[16];
    double T23[16];
    double T34[16];
    double T45[16];
    double T56[16];
    double T67[16];

    /* CoordinateTrans(alpha_0,a_0,theta_1,d_1),MDH的坐标系转换公式 */
    CoordinateTrans(0.0, 0.0, angle[0], 0.0, T01);
    CoordinateTrans(1.5707963267948966, 0.0, angle[1], 0.0, T12);
    CoordinateTrans(-1.5707963267948966, 0.0, angle[2], 449.5, T23);
    CoordinateTrans(1.5707963267948966, 30.0, angle[3], 0.0, T34);
    CoordinateTrans(-1.5707963267948966, -30.0, angle[4], 449.5, T45);
    CoordinateTrans(1.5707963267948966, 0.0, angle[5], 0.0, T56);
    CoordinateTrans(-1.5707963267948966, 0.0, angle[6], 0.0, T67);

    double temp[16];
    double temp2[16];
    matrixMultiply(T01, 4, 4, T12, 4, 4, temp);
    matrixMultiply(temp, 4, 4, T23, 4, 4, temp2);
    matrixMultiply(temp2, 4, 4, T34, 4, 4, temp);
    matrixMultiply(temp, 4, 4, T45, 4, 4, temp2);
    matrixMultiply(temp2, 4, 4, T56, 4, 4, temp);
    matrixMultiply(temp, 4, 4, T67, 4, 4, T07);

int i;
    // for(i = 0; i< 16; i++){
    //     printf(",%f" + !i, temp2[i]);
    // }
    // printf("\n");
}

/*
 * 功能------------逆运动学+冗余自由度规划
 *                      workMode=0时：betaInit=上一个β值；
 *                      workMode=1时：betaInit=任意值，随便给一个；
 *                      workMode=2时：betaInit = 给定值；
 * 输入------------angleInit：初始关节角度1x7
 *                 expectPose：期望位姿4x4
 *                 betaInit：初始β值 
 *                 betaScanInterval：扫描间隔（精度)
 *                 betaEnd：终止β值
 * 输出------------AngleByPlanning：逆运动学求解出的位姿对应关节角1x7
 */
void InverseKinematics(const double angleInit[7], const double expectPose[16], double betaInit, double betaScanInterval, double betaEnd,
                       double angleByPlanning_data[], int angleByPlanning_size[2])
{
    static const double dv4[3] = {-1.0, 0.0, 0.0};                       // 中间变量，垂直向上向量
    const double dv0[3] = {448.50221975582684, 29.933407325194224, 0.0}; /* P_K_E0 求坐标系K下点坐标系3原点E0的坐标*/

    double evaluation_final;
    int flagOfFirstCirculation;
    double n[3];
    double d5;
    double length_BW;
    double theta_B;
    double R0k[9];
    double RkK[9];
    double theta_5;
    double theta_E;
    double betaDown;
    double betaUp;
    double betaCount;
    int i;
    bool exitg1;
    double b_expectPose[3];
    double rotationMatrix[9];
    int i22;
    double z4[3];
    double b_z4[3];
    int i23;
    double b_R0k[9];
    int flag;
    double P_0_E0[3];
    double x_tmp;
    int theta_2;
    double b_theta_2;
    double theta_1;
    double d6;
    double theta_3;
    double T01_inverse[16];
    double T12_inverse[16];
    double T23_inverse[16];
    double T34_inverse[16];
    double T47_inverse[16];
    int i24;
    int T01_inverse_tmp;
    double b_x_tmp;
    double b_T01_inverse[16];
    double theta_new[7];
    double b_theta_new[8];
    evaluation_final = 0.0;

    flagOfFirstCirculation = 0; /* β取值循环的第一次循环标志，用于初始化评价值及最优估计值 */

    // ==================================================================//
    // ===================== 逆运动学求解前四个关节值 ===================== //
    // ==================================================================//

    //  /* （1）由W的位置求解p轴向量 */
    // (2)由BW与竖直向上向量(-1,0,0)求解零位面的法向量n
    cross(*(double(*)[3]) & expectPose[12], dv4, n);

    d5 = b_norm(n);
    n[0] /= d5;
    n[1] /= d5;
    n[2] /= d5;

    /* (41)求解初始姿态下的各关节长度length_BE0、length_EW0及肘关节角度theta_E0 */
    /* d3,a3为MDH中的参数 */
    /* 基坐标系原点与坐标系4原点E距离，固定值 */
    /* 基坐标系原点B与坐标系3原点E'距离 */
    /* ∠EBE'机械臂设计结构上固有夹角 */
    /* 机械臂设计结构上固有夹角 */

    /* (42)求解期望姿态下的△BEW参数*/
    length_BW = b_norm(*(double(*)[3]) & expectPose[12]);
    theta_B = acos(length_BW / 901.0);

    /* 43求基坐标系0到坐标系k的姿态矩阵R0k */
    memset(&R0k[0], 0, 9U * sizeof(double));
    R0k[0] = expectPose[12] / length_BW;
    R0k[6] = n[0];
    R0k[1] = expectPose[13] / length_BW;
    R0k[7] = n[1];
    R0k[2] = expectPose[14] / length_BW;
    R0k[8] = n[2];
    R0k[3] = n[1] * R0k[2] - n[2] * R0k[1];
    R0k[4] = n[2] * R0k[0] - n[0] * R0k[2];
    R0k[5] = n[0] * R0k[1] - n[1] * R0k[0];

    /* （44）求坐标系k到坐标系K的姿态转移矩阵 */
    /* 绕zk轴旋转theta_B〉〉〉〉这里和θ4取正值时相反 */
    RkK[2] = 0.0;
    RkK[5] = 0.0;
    RkK[8] = 1.0;
    theta_5 = cos(theta_B);
    RkK[0] = theta_5;
    theta_B = sin(theta_B);
    RkK[3] = -theta_B;
    RkK[6] = 0.0;
    RkK[1] = theta_B;
    RkK[4] = theta_5;
    RkK[7] = 0.0;

    /* （45）求坐标系K下点坐标系3原点E0的坐标 */
    /* (7)通过余弦定理解∠BEW，从而解theta4 这里和θ4取正值时不同*/

    theta_E = acos((405900.5 - length_BW * length_BW) / 405900.5);

    /* 根据输入β进行计算 */

    if (betaInit > betaEnd)
    {
        betaUp = betaInit;
        betaDown = betaEnd;
    }
    else
    {
        betaUp = betaEnd;
        betaDown = betaInit;
    }
    if (betaScanInterval == 0.0f)
    { //如果间隔为零，则赋值为1，防止除零
        betaScanInterval = 1.0f;
    }

    betaCount = ceil((betaUp - betaDown) / betaScanInterval); // 向上取整，最后一个beta单独赋值

    // =============== 使用评价函数进行冗余自由度规划 ================ //
    i = 0;
    while (i <= (int)(betaCount))
    {
        if (i == (int)betaCount)
        {
            betaDown = betaUp;
        }

        b_expectPose[0] = expectPose[12];
        b_expectPose[1] = expectPose[13];
        b_expectPose[2] = expectPose[14];
        VectorRot(b_expectPose, betaDown, rotationMatrix);

        /* (3)n绕p轴旋转β角度得到z4轴,此时旋转矩阵为4×4 */
        for (i22 = 0; i22 < 3; i22++)
        {
            z4[i22] = 0.0;
            z4[i22] = (rotationMatrix[i22] * n[0] + rotationMatrix[i22 + 3] * n[1]) +
                      rotationMatrix[i22 + 6] * n[2];
        }

        /* 此时求出的z4维度为1×3，需要转置 */
        d5 = b_norm(z4);

        /* （46）求坐标系3原点E0在基坐标系下的表示(注意此时机械臂仍在竖直平面） */
        for (i22 = 0; i22 < 3; i22++)
        {
            b_z4[i22] = z4[i22] / d5;
            b_expectPose[i22] = 0.0;
            for (i23 = 0; i23 < 3; i23++)
            {
                b_expectPose[i22] += ((R0k[i22] * RkK[3 * i23] + R0k[i22 + 3] * RkK[1 + 3 * i23]) + R0k[i22 + 6] * RkK[2 + 3 * i23]) * dv0[i23];
            }
        }

        for (i22 = 0; i22 < 3; i22++)
        {
            z4[i22] = 0.0;
            for (i23 = 0; i23 < 3; i23++)
            {
                flag = i22 + 3 * i23;
                b_R0k[flag] = 0.0;
                b_R0k[flag] = (R0k[i22] * RkK[3 * i23] + R0k[i22 + 3] * RkK[1 + 3 * i23]) + R0k[i22 + 6] * RkK[2 + 3 * i23];
                z4[i22] += rotationMatrix[flag] * b_expectPose[i23];
            }
        }

        for (i22 = 0; i22 < 3; i22++)
        {
            b_expectPose[i22] = 0.0;
            b_expectPose[i22] = (b_R0k[i22] * 448.50221975582684 + b_R0k[i22 + 3] * 29.933407325194224) + b_R0k[i22 + 6] * 0.0;
        }

        for (i22 = 0; i22 < 3; i22++)
        {
            P_0_E0[i22] = (rotationMatrix[i22] * b_expectPose[0] + rotationMatrix[i22 + 3] * b_expectPose[1]) + rotationMatrix[i22 + 6] * b_expectPose[2];
        }

        /* （47）将E0绕BW旋转β角度，得到真正的期望位姿下坐标系3的原点在基坐标系下的表示 */

        /* （5）通过E0点位置解前两个关节theta1和theta2 */
        /* 由于arccos的值域为0~π，需要设计程序使theta_2取值为正负arccos */
        x_tmp = acos(z4[2] / 449.5);
        d5 = 2.0 * x_tmp;
        for (theta_2 = 0; theta_2 < 2; theta_2++)
        {
            b_theta_2 = -x_tmp + (double)theta_2 * d5;

            if (!(b_theta_2 == 0.0))
            {
                length_BW = sin(b_theta_2);
                theta_B = 449.5 * length_BW;
                theta_1 = rt_atan2d_snf(-P_0_E0[1] / theta_B, -P_0_E0[0] / theta_B);
                d6 = cos(theta_1);
                /* （6）通过z4轴方向解第三个关节 theta3 */

                if (d6 == 0.0)
                {
                    theta_3 = rt_atan2d_snf(b_z4[2] / sin(b_theta_2), b_z4[0] / sin(theta_1));
                }
                else
                {
                    theta_3 = rt_atan2d_snf(b_z4[2] / length_BW, (-b_z4[1] + sin(theta_1) * cos(b_theta_2) * b_z4[2] / length_BW) / d6);
                }

                // ==================================================================//
                // ===================== 逆运动学求解后三个关节值 ===================== //
                // ==================================================================//

                CoordinateTrans(0.0, 0.0, theta_1, 0.0, T01_inverse);
                CoordinateTrans(1.5707963267948966, 0.0, b_theta_2, 0.0, T12_inverse);
                CoordinateTrans(-1.5707963267948966, 0.0, theta_3, 449.5, T23_inverse);
                CoordinateTrans(1.5707963267948966, 30.0, (3.0083086700768047 + theta_E) - 6.2831853071795862, 0.0, T34_inverse);

                /* 将正运动学末端位姿设定为期望末端位姿 */
                memcpy(&T47_inverse[0], &expectPose[0], sizeof(double) << 4);
                double temp[16];
                double temp2[16];
                matrixMultiply(T01_inverse, 4, 4, T12_inverse, 4, 4, temp);
                matrixMultiply(temp, 4, 4, T23_inverse, 4, 4, temp2);
                matrixMultiply(temp2, 4, 4, T34_inverse, 4, 4, temp);

                mldivide(temp, T47_inverse); // T47_inverse = T04_inverse(temp)\T07_inverse(T47_inverse = expectPose);
                if (!(T47_inverse[9] == 0.0))
                {
                    /* 由于arccos的值域为0~π，需要设计程序取使theta_6取值为正负acos */
                    b_x_tmp = acos(T47_inverse[9]);
                    d6 = 2.0 * b_x_tmp;
                    for (T01_inverse_tmp = 0; T01_inverse_tmp < 2; T01_inverse_tmp++)
                    {
                        length_BW = -b_x_tmp + (double)T01_inverse_tmp * d6; // theta6

                        theta_B = sin(length_BW);
                        theta_5 = rt_atan2d_snf(T47_inverse[10] / theta_B, -T47_inverse[8] / theta_B);
                        theta_B = rt_atan2d_snf(-T47_inverse[5] / theta_B, T47_inverse[1] / theta_B);

                        // ===================== 角度限制验证 ===================== //
                        flag = 0;
                        if ((theta_1 <= -2.967) || (theta_1 >= 2.967))
                        {
                            /* 第一个关节超出范围，请重新选择β或期望位姿 */
                            flag = 1;
                        }

                        if ((b_theta_2 <= -2.094) || (b_theta_2 >= 2.094))
                        {
                            /* 第二个关节超出范围，请重新选择β或期望位姿 */
                            flag = 1;
                        }

                        if ((theta_3 <= -2.967) || (theta_3 >= 2.967))
                        {
                            /* 第三个关节超出范围，请重新选择β或期望位姿 */
                            flag = 1;
                        }

                        if (((3.0083086700768047 + theta_E) - 6.2831853071795862 <= -2.792) ||
                            ((3.0083086700768047 + theta_E) - 6.2831853071795862 >= 2.094))
                        {
                            /* 第四个关节超出范围，请重新选择期望位姿*/
                            flag = 1;
                        }

                        if ((theta_5 <= -2.967) || (theta_5 >= 2.967))
                        {
                            /* 第五个关节超出范围，请重新选择期望位姿*/
                            flag = 1;
                        }

                        if ((length_BW <= -2.181) || (length_BW >= 2.181))
                        {
                            /*第六个关节超出范围，请重新选择期望位姿 */
                            flag = 1;
                        }

                        if ((theta_B <= -2.967) || (theta_B >= 2.967))
                        {
                            /*第七个关节超出范围，请重新选择期望位姿*/
                            flag = 1;
                        }

                        if (flag == 0)
                        {
                            // ===================== 利用评价函数筛选最优规划角度 ===================== //

                            theta_new[0] = theta_1;
                            theta_new[1] = b_theta_2;
                            theta_new[2] = theta_3;
                            theta_new[3] = (3.0083086700768047 + theta_E) - 6.2831853071795862;
                            theta_new[4] = theta_5;
                            theta_new[5] = length_BW;
                            theta_new[6] = theta_B;
                            theta_B = 7.0 * fabs(theta_1 - angleInit[0]) + 6.0 * fabs(b_theta_2 - angleInit[1]) + 5.0 * fabs(theta_3 - angleInit[2]) + 4.0 * fabs(((3.0083086700768047 + theta_E) - 6.2831853071795862) - angleInit[3]) + 3.0 * fabs(theta_5 - angleInit[4]) + 2.0 * fabs(length_BW - angleInit[5]) + 1.0 * fabs(theta_B - angleInit[6]);

                            /* 将第一次循环后的评价值存入评价函数；第一次规划值存入theta_final，第二次及以后跳过此循环 */
                            if (flagOfFirstCirculation == 0)
                            {
                                evaluation_final = theta_B;
                                for (i24 = 0; i24 < 7; i24++)
                                {
                                    b_theta_new[i24] = theta_new[i24];
                                }

                                b_theta_new[7] = betaDown;
                                angleByPlanning_size[0] = 1;
                                angleByPlanning_size[1] = 8;
                                memcpy(&angleByPlanning_data[0], &b_theta_new[0], sizeof(double) << 3);

                                /* 将β放入解中验证机械臂是否位置突变*/
                                flagOfFirstCirculation = 1;
                            }

                            if (theta_B < evaluation_final)
                            {
                                /* 如果发现更小的评价值，存储此评价值及最优估计值 */
                                evaluation_final = theta_B;
                                for (i24 = 0; i24 < 7; i24++)
                                {
                                    b_theta_new[i24] = theta_new[i24];
                                }

                                /* 将β放入解中以便验证机械臂位置是否突变 */
                                b_theta_new[7] = betaDown;
                                angleByPlanning_size[0] = 1;
                                angleByPlanning_size[1] = 8;
                                memcpy(&angleByPlanning_data[0], &b_theta_new[0], sizeof(double) << 3);
                            }
                        }
                        else
                        {
                            /* 如果存在以上任何一种冲突情况，停止此次规划，进入下一次循环 */
                        }

                    } /* theta6取正负值循环结束*/

                } // if (!(T47_inverse[9] == 0.0))
                else
                {
                    /* T47_inverse(2,3)的表达式为c6，首先判断是否为0*/
                    printf("关节奇异，请重新选定β\n");
                }

            } // if (!(b_theta_2 == 0.0))
            else
            {
                /* 当第二个关节规划角度为0时，1、3关节奇异，剔除该取值*/
                printf("关节奇异，请重新规划\n");
            }

        } /* theta2取正负值循环结束*/

        betaDown += betaScanInterval;
        i++;

    } // 使用评价函数进行冗余自由度规划
}

/*
 * 功能------------------计算出当前关节角所对应的β角，另外可用求两法向量夹角计算，这里直接套用的逆运动学部分代码，运算速度较低
 * 输入------------------angle：当前关节角1x7
 * 输出------------------findBeta：当前关节角所对应的β角
 */
double FindBetaSlow(const double angle[7])
{
    double dv0[3] = {448.50221975582684, 29.933407325194224, 0.0};
    static const double dv4[3] = {-1.0, 0.0, 0.0};

    double findBeta;
    double error_final;
    double T01[16];
    double T12[16];
    double T23[16];
    double T03[16];
    double EndPose[16];
    int i4;
    int i5;
    int flagOfFirstCirculation;
    int T01_tmp;
    double n[3];
    double length_BW;
    double theta_B;
    double R0k[9];
    double RkK[9];
    double beta;
    double rotationMatrix[9];
    double b_R0k[3];
    double b_a;
    findBeta = 0.0;
    error_final = 0.0;

    /* CoordinateTrans(alpha_0,a_0,theta_1,d_1)*/
    CoordinateTrans(0.0, 0.0, angle[0], 0.0, T01);
    CoordinateTrans(1.5707963267948966, 0.0, angle[1], 0.0, T12);
    CoordinateTrans(-1.5707963267948966, 0.0, angle[2], 449.5, T23);

    /*  T03 = T01*T12*T23; */
    double temp[16];
    matrixMultiply(T01, 4, 4, T12, 4, 4, temp);
    matrixMultiply(temp, 4, 4, T23, 4, 4, T03);

    ForwardKinematics(angle, EndPose);
    
    /*  简化计算 */
    double BE[3], BW[3], res[3], res2[3];
    memcpy(BE, *(double(*)[3]) & T03[12], sizeof(BE));
    memcpy(BW, *(double(*)[3]) & EndPose[12],sizeof(BW));
  
    cross(BE, BW, res);
    memcpy(BE, BW, sizeof(BW));
    BE[2] = 0.0;
    cross(BW, BE, res2);

    double len1 = b_norm(res);
    double len2 = b_norm(res2);
    int i;
    for(i = 0; i < 3; i ++)
    {
        res[i] /= len1;
        res2[i] /= len2;
    }
    double angle_test = acos(dot(res, res2));
    /*  简化计算 */


    cross(*(double(*)[3]) & EndPose[12], dv4, n);

    length_BW = b_norm(n);
    n[0] /= length_BW;
    n[1] /= length_BW;
    n[2] /= length_BW;

    length_BW = sqrt((EndPose[12] * EndPose[12] + EndPose[13] * EndPose[13]) + EndPose[14] * EndPose[14]);

    theta_B = acos(length_BW / 901.0);
    memset(&R0k[0], 0, 9U * sizeof(double));

    R0k[0] = EndPose[12] / length_BW;
    R0k[1] = EndPose[13] / length_BW;
    R0k[2] = EndPose[14] / length_BW;
    R0k[6] = n[0];
    R0k[7] = n[1];
    R0k[8] = n[2];
    R0k[3] = n[1] * R0k[2] - n[2] * R0k[1];
    R0k[4] = n[2] * R0k[0] - n[0] * R0k[2];
    R0k[5] = n[0] * R0k[1] - n[1] * R0k[0];

    RkK[0] = cos(theta_B);
    RkK[1] = sin(theta_B);
    RkK[2] = 0.0;
    RkK[3] = -sin(theta_B);
    RkK[4] = cos(theta_B);
    RkK[5] = 0.0;
    RkK[6] = 0.0;
    RkK[7] = 0.0;
    RkK[8] = 1.0;

    flagOfFirstCirculation = 0;

    beta = -3.1415926535897931;
    for (T01_tmp = 0; T01_tmp < 62833; T01_tmp++)
    {
        if (T01_tmp == 62833 - 1)
        {
            beta = 3.1415926535897931;
        }

        n[0] = EndPose[12];
        n[1] = EndPose[13];
        n[2] = EndPose[14];
        VectorRot(n, beta, rotationMatrix);

        matrixMultiply(R0k, 3, 3, RkK, 3, 3, temp);
        matrixMultiply(temp, 3, 3, dv0, 3, 1, b_R0k);

        for (i4 = 0; i4 < 3; i4++)
        {
            n[i4] = (rotationMatrix[i4] * b_R0k[0] + rotationMatrix[i4 + 3] * b_R0k[1]) + rotationMatrix[i4 + 6] * b_R0k[2];
        }

        length_BW = T03[12] - n[0];
        theta_B = T03[13] - n[1];
        b_a = T03[14] - n[2];
        length_BW = length_BW * length_BW + theta_B * theta_B + b_a * b_a;

        if (flagOfFirstCirculation == 0)
        {
            error_final = length_BW;
            findBeta = beta;
            flagOfFirstCirculation = 1;
        }

        if (length_BW < error_final)
        {
            error_final = length_BW;
            findBeta = beta;
        }

        beta += 0.0001;
    }

    return findBeta;
}


/*
 * 功能------------------计算出当前关节角所对应的β角，通过两法向量夹角计算
 * 输入------------------angle：当前关节角1x7
 * 输出------------------findBeta：当前关节角所对应的β角
 * 转C初始化要求
 * Arguments    : const double angle[7]
 * Return Type  : double
 */
double FindBeta(const double angle[7])
{
  double findBeta;
  double T01[16];
  double T12[16];
  double T23[16];
  int i0;
  int i1;
  int T01_tmp;
  int b_T01_tmp;
  double P0E_idx_0;
  double b_T01[16];
  double P0E_idx_1;
  double P0E_idx_2;
  double T03[16];
  double nVertical_idx_0;
  double nVertical_idx_1;
  double nVertical_idx_2;
  double nReal_idx_0;
  double temp1[16];
  double temp2[16];
  CoordinateTrans(0.0, 0.0, angle[0], 0.0, T01);
  CoordinateTrans(1.5707963267948966, 0.0, angle[1], 0.0, T12);
  CoordinateTrans(-1.5707963267948966, 0.0, angle[2], 449.5, T23);

  matrixMultiply(T01, 4, 4, T12, 4, 4, temp1);
  matrixMultiply(temp1, 4, 4, T23, 4, 4, T03);

  P0E_idx_0 = T03[12];
  P0E_idx_1 = T03[13];
  P0E_idx_2 = T03[14];

  /* 已知BW的前提下，计算假设机械臂平面在竖直平面下，E的坐标 */
  ForwardKinematics(angle, T01);

  /* TCP中心点坐标，同时为基坐标系指向该点的向量1x3 */
  nVertical_idx_0 = T01[13] * 0.0 - T01[14] * 0.0;
  nVertical_idx_1 = -T01[14] - T01[12] * 0.0;
  nVertical_idx_2 = T01[12] * 0.0 - (-T01[13]);

  /* (2)TCP中心点与竖直向上向量(-1,0,0)求解零位面的法向量n  */
  nReal_idx_0 = P0E_idx_1 * T01[14] - P0E_idx_2 * T01[13];
  P0E_idx_2 = P0E_idx_2 * T01[12] - P0E_idx_0 * T01[14];
  P0E_idx_0 = P0E_idx_0 * T01[13] - P0E_idx_1 * T01[12];
  P0E_idx_0 = ((nVertical_idx_0 * nReal_idx_0 + nVertical_idx_1 * P0E_idx_2) +
               nVertical_idx_2 * P0E_idx_0) / (sqrt((nVertical_idx_0 *
    nVertical_idx_0 + nVertical_idx_1 * nVertical_idx_1) + nVertical_idx_2 *
    nVertical_idx_2) * sqrt((nReal_idx_0 * nReal_idx_0 + P0E_idx_2 * P0E_idx_2)
    + P0E_idx_0 * P0E_idx_0));

  if (P0E_idx_0 > 1.0) {
    P0E_idx_0 = 1.0;
  } else {
    if (P0E_idx_0 < -1.0) {
      P0E_idx_0 = -1.0;
    }
  }

  P0E_idx_0 = acos(P0E_idx_0);

  /* y依据E点所在空间象限确定beta的符号 */
  if ((T01[12] == T01[13]) && (T01[13] == T01[14]) && (T01[12] == 0.0)) {
    /* disp('TCP中心点不能和原点重合'); */
    findBeta = 10.0;

    /* 这里还需要改 */
  } else if (T01[14] == 0.0) {
    /* 此时直线BW在xy平面,排除P0W(3)=0的情况，使y的除数不为0 */
    if (T03[14] >= 0.0) {
      findBeta = -(3.1415926535897931 - P0E_idx_0);
    } else {
      findBeta = 3.1415926535897931 - P0E_idx_0;
    }
  } else {
    P0E_idx_2 = T03[13] - T01[13] / T01[14] * T03[14];

    /* y为正表示点在平面上方 */
    if (fabs(P0E_idx_2) < 0.0001) {
      /* 浮点数运算在0附近可能会出错，在一定范围强制为0 */
      P0E_idx_2 = 0.0;
    }

    if (P0E_idx_2 >= 0.0) {
      findBeta = -(3.1415926535897931 - P0E_idx_0);
    } else {
      findBeta = 3.1415926535897931 - P0E_idx_0;
    }
  }

  return findBeta;
}

void InverseKinematicsHead(double RwChest[9], double mapAngle[3])
{
    double cs;
    // 依据R03求解头部三个关节
    mapAngle[1] = asin(-RwChest[8]);
    cs = cos(mapAngle[1]);
    
    // 横滚角一般人达不到90度，不用考虑cos等于0的情况
    mapAngle[0] = rt_atan2d_snf(-RwChest[7] / cs, -RwChest[6] / cs);
    mapAngle[2] = rt_atan2d_snf(RwChest[2] / cs, RwChest[5] / cs);
}


void posfromT(double T[16], double pos[3])
{
    int i;
    for(i = 0; i < 3; i++)
    {
        pos[i] = T[i + 12];
    }
}

void rotfromT(double T[16], double rot[9])
{
    int i;
    for(i = 0; i < 3; i++)
    {
        rot[i * 3 + 0] = T[i * 4 + 0];
        rot[i * 3 + 1] = T[i * 4 + 1];
        rot[i * 3 + 2] = T[i * 4 + 2];
    }
}

void TfromRotPos(const double rot[9], const double pos[3], double trans[16])
{
    int i;
    for(i = 0; i < 3; i++)
    {
        trans[i * 4 + 0] = rot[i * 3 + 0];
        trans[i * 4 + 1] = rot[i * 3 + 1];
        trans[i * 4 + 2] = rot[i * 3 + 2];
        trans[i * 4 + 3] = 0.0;
        trans[i + 12] = pos[i];
    }
    trans[15] = 1.0;
}

void TfromPose(double pose[6], double T[16])   // ZYX 顺规 先旋转x 后y， 后z R = Rz*Ry*Rx
{
    double sr = sin(pose[3]);    // 弧度
    double cr = cos(pose[3]);
    double sp = sin(pose[4]);
    double cp = cos(pose[4]);
    double sy = sin(pose[5]);
    double cy = cos(pose[5]);
    double Rx[9] = {1, 0, 0, 0, cr, sr, 0, -sr, cr};
    double Ry[9] = {cp, 0, -sp, 0, 1, 0, sp, 0, cp};
    double Rz[9] = {cy, sy, 0, -sy, cy, 0, 0, 0, 1};
    double t1[9],t2[9];
    
    matrixMultiply(Rz,3,3,Ry,3,3,t1);
    matrixMultiply(t1,3,3,Rx,3,3,t2);
    TfromRotPos(t2, *(double(*)[3])&pose[0], T);
}

void PosefromT(double T[16], double pose[6])
{
    double rot[9], rot_[9], I[9];
    int i ;
    double sum;

    rotfromT(T, rot);
    matrixTrans(rot, 3, 3, rot_);
    matrixMultiply(rot, 3, 3, rot_, 3, 3, I);
    sum = 0.0;
    for ( i = 0; i< 9; i++)
    {
        if (i % 4 == 0)
        {
            sum += (1.0 - I[i]) * (1.0 - I[i]);
        }
        else
        {
            sum += I[i] * I[i];
        }
    }
    sum = sqrt(sum);
    if (sum > 1e-3)
    {
        pose[3] = 0;
        pose[4] = 0;
        pose[5] = 0;
        printf("Invalid rotation matrix\n");
    }

    double sy = sqrt(rot[0] * rot[0] + rot[1] * rot[1]);
    bool singular = (sy < 1e-6);
    double x, y, z;
    if (!singular)
    {
        pose[3] = atan2( rot[5], rot[8]);
        pose[4] = atan2(-rot[2], sy);
        pose[5] = atan2( rot[1], rot[0]);
    }
    else
    {
        pose[3] = atan2(-rot[7], rot[4]);
        pose[4] = atan2(-rot[2], sy);
        pose[5] = 0;
    }

    pose[0] = T[12];
    pose[1] = T[13];
    pose[2] = T[14];

}

void Jacobian(double jointangle[6], double J[42])
{
    double x1 = jointangle[0];
    double x2 = jointangle[1];
    double x3 = jointangle[2];
    double x4 = jointangle[3];
    double x5 = jointangle[4];
    double x6 = jointangle[5];
    double x7 = jointangle[6];
    
    double a = 30;
    double d = 449.5;

    double jv0[21];
    double jv7[21];

    int i, j;

    jv0[0 + 0 * 3] = a*(cos(x4)*(cos(x1)*sin(x3)+sin(x1)*cos(x2)*cos(x3))-sin(x1)*sin(x2)*sin(x4))+d*(sin(x4)*(cos(x1)*sin(x3)+sin(x1)*cos(x2)*cos(x3))+sin(x1)*sin(x2)*cos(x4))-a*(cos(x1)*sin(x3)+sin(x1)*cos(x2)*cos(x3))+d*sin(x1)*sin(x2);
	jv0[0 + 1 * 3] = a*(cos(x4)*cos(x1)*sin(x2)*cos(x3)+cos(x1)*cos(x2)*sin(x4))+d*(sin(x4)*cos(x1)*sin(x2)*cos(x3)-cos(x1)*cos(x2)*cos(x4))-a*cos(x1)*sin(x2)*cos(x3)-d*cos(x1)*cos(x2);
	jv0[0 + 2 * 3] = a*(cos(x4)*(sin(x1)*cos(x3)+cos(x1)*cos(x2)*sin(x3)))+d*(sin(x4)*(sin(x1)*cos(x3)+cos(x1)*cos(x2)*sin(x3)))-a*(sin(x1)*cos(x3)+cos(x1)*cos(x2)*sin(x3));
	jv0[0 + 3 * 3] = a*(-sin(x4)*(sin(x1)*sin(x3)-cos(x1)*cos(x2)*cos(x3))+cos(x1)*sin(x2)*cos(x4))+d*(cos(x4)*(sin(x1)*sin(x3)-cos(x1)*cos(x2)*cos(x3))+cos(x1)*sin(x2)*sin(x4));
	jv0[0 + 4 * 3] = 0;
	jv0[0 + 5 * 3] = 0;
	jv0[0 + 6 * 3] = 0;

	jv0[1 + 0 * 3] =	a*(-sin(x1)*sin(x3)+cos(x1)*cos(x2)*cos(x3))-d*(sin(x4)*(-sin(x1)*sin(x3)+cos(x1)*cos(x2)*cos(x3))+cos(x1)*sin(x2)*cos(x4))-a*(cos(x4)*(-sin(x1)*sin(x3)+cos(x1)*cos(x2)*cos(x3))-cos(x1)*sin(x2)*sin(x4))-d*cos(x1)*sin(x2);
	jv0[1 + 1 * 3] = -a*sin(x1)*sin(x2)*cos(x3)-d*(-sin(x4)*sin(x1)*sin(x2)*cos(x3)+sin(x1)*cos(x2)*cos(x4))-a*(-cos(x4)*sin(x1)*sin(x2)*cos(x3)-sin(x1)*cos(x2)*sin(x4))-d*sin(x1)*cos(x2);
	jv0[1 + 2 * 3] = a*(cos(x1)*cos(x3)-sin(x1)*cos(x2)*sin(x3))-d*sin(x4)*(cos(x1)*cos(x3)-sin(x1)*cos(x2)*sin(x3))-a*cos(x4)*(cos(x1)*cos(x3)-sin(x1)*cos(x2)*sin(x3));
	jv0[1 + 3 * 3] =	-d*(cos(x4)*(cos(x1)*sin(x3)+sin(x1)*cos(x2)*cos(x3))-sin(x1)*sin(x2)*sin(x4))-a*(-sin(x4)*(cos(x1)*sin(x3)+sin(x1)*cos(x2)*cos(x3))-sin(x1)*sin(x2)*cos(x4));       
	jv0[1 + 4 * 3] = 0;
	jv0[1 + 5 * 3] = 0;
	jv0[1 + 6 * 3] = 0;

	jv0[2 + 0 * 3] = 0;
	jv0[2 + 1 * 3] = -d*sin(x2)-a*(-sin(x2)*sin(x4)+cos(x2)*cos(x3)*cos(x4))+d*(-sin(x2)*cos(x4)-cos(x2)*cos(x3)*sin(x4))+a*cos(x2)*cos(x3);
	jv0[2 + 2 * 3] = a*sin(x2)*sin(x3)*cos(x4)+d*sin(x2)*sin(x3)*sin(x4)-a*sin(x2)*sin(x3);        
	jv0[2 + 3 * 3] = -a*(cos(x2)*cos(x4)-sin(x2)*cos(x3)*sin(x4))+d*(-cos(x2)*sin(x4)-sin(x2)*cos(x3)*cos(x4)); 
	jv0[2 + 4 * 3] = 0;
	jv0[2 + 5 * 3] = 0;
	jv0[2 + 6 * 3] = 0;

    double T07[16];
    double rot[9], rot_[9];
    ForwardKinematics(jointangle, T07);
    rotfromT(T07, rot);
    matrixTrans(rot, 3, 3, rot_);

    matrixMultiply(rot_, 3, 3, jv0, 3, 7, jv7);

    double j7w[21];

    j7w[0 + 0 * 3] = 0;  	
	j7w[1 + 0 * 3] = 0; 	
	j7w[2 + 0 * 3] = 1; 	

	j7w[0 + 1 * 3] = sin(x1);
	j7w[1 + 1 * 3] = -cos(x1);	
	j7w[2 + 1 * 3] = 0; 

	j7w[0 + 2 * 3] = -cos(x1)*sin(x2); 	
	j7w[1 + 2 * 3] = -sin(x1)*sin(x2); 	
	j7w[2 + 2 * 3] = cos(x2); 

	j7w[0 + 3 * 3] = sin(x1)*cos(x3)+cos(x1)*cos(x2)*sin(x3); 	
	j7w[1 + 3 * 3] = sin(x1)*cos(x2)*sin(x3)-cos(x1)*cos(x3); 	
	j7w[2 + 3 * 3] = sin(x2)*sin(x3); 	

	j7w[0 + 4 * 3] = sin(x4)*(sin(x1)*sin(x3)-cos(x1)*cos(x2)*cos(x3))-cos(x1)*sin(x2)*cos(x4); 	
	j7w[1 + 4 * 3] = -sin(x4)*(cos(x1)*sin(x3)+sin(x1)*cos(x2)*cos(x3))-sin(x1)*sin(x2)*cos(x4); 	
	j7w[2 + 4 * 3] = cos(x2)*cos(x4)-sin(x2)*cos(x3)*sin(x4); 	

	j7w[0 + 5 * 3] = cos(x5)*(sin(x1)*cos(x3)+cos(x1)*cos(x2)*sin(x3))-sin(x5)*(cos(x4)*(sin(x1)*sin(x3)-cos(x1)*cos(x2)*cos(x3))+cos(x1)*sin(x2)*sin(x4));  	
	j7w[1 + 5 * 3] = sin(x5)*(cos(x4)*(cos(x1)*sin(x3)+sin(x1)*cos(x2)*cos(x3))-sin(x1)*sin(x2)*sin(x4))-cos(x5)*(cos(x1)*cos(x3)-sin(x1)*cos(x2)*sin(x3)); 	
	j7w[2 + 5 * 3] = sin(x5)*(cos(x2)*sin(x4)+sin(x2)*cos(x3)*cos(x4))+sin(x2)*sin(x3)*cos(x5); 	

	j7w[0 + 6 * 3] = cos(x6)*(sin(x4)*(sin(x1)*sin(x3)-cos(x1)*cos(x2)*cos(x3))-cos(x1)*sin(x2)*cos(x4))+sin(x6)*(cos(x5)*(cos(x4)*(sin(x1)*sin(x3)-cos(x1)*cos(x2)*cos(x3))+cos(x1)*sin(x2)*sin(x4))+sin(x5)*(sin(x1)*cos(x3)+cos(x1)*cos(x2)*sin(x3))); 	
	j7w[1 + 6 * 3] = -cos(x6)*(sin(x4)*(cos(x1)*sin(x3)+sin(x1)*cos(x2)*cos(x3))+sin(x1)*sin(x2)*cos(x4))-sin(x6)*(cos(x5)*(cos(x4)*(cos(x1)*sin(x3)+sin(x1)*cos(x2)*cos(x3))-sin(x1)*sin(x2)*sin(x4))+sin(x5)*(cos(x1)*cos(x3)-sin(x1)*cos(x2)*sin(x3))); 	
	j7w[2 + 6 * 3] = cos(x6)*(cos(x2)*cos(x4)-sin(x2)*cos(x3)*sin(x4))-sin(x6)*(cos(x5)*(cos(x2)*sin(x4)+sin(x2)*cos(x3)*cos(x4))-sin(x2)*sin(x3)*sin(x5)); 

    for ( i = 0; i < 3; i ++)
    {
        for (j = 0; j< 7; j++)
        {
            J[j * 6 + i] = jv7[ j * 3 + i];
        }
    }

    for ( i = 0; i < 3; i ++)
    {
        for (j = 0; j< 7; j++)
        {
            J[j * 6 + i + 3] = j7w[ j * 3 + i];
        }
    }

}