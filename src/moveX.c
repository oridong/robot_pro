#include <math.h>
#include <stdio.h>
#include <string.h>
#include "math_tool.h"
#include "kinematics.h"
#include "S_trajectory.h"
#include "ThreePointCircle.h"
#include "control.h"
#include "rt_type/tmwtypes.h"

/*
 * 功能： 大范围关节角转动（关节空间）
 * 输入-------------angleInit：初始关节角1x7
 *                  poseOrjointFinal：最终位姿4x4，需要先初始化为零
 *                  speedRate：机械臂最大速度比例，取值0~1，用于调节机械臂最大速度
 * 输出-------------angleByPlanningL：返回多组离散关节角，每0.01s发送给驱动器一组
 */
void moveJ(const double angleInit[7], const double poseOrjointFinal[16], double speedRate, bodypart arm)
{
    double angleFianl_beta[8];
    int angleFianl_beta_size[2];
    double c_Tmax;
    int i;
    double t;
    double para[17];
    int i17;
    int i18;
    double b_para[112];
    double T[7];
    int loop_ub;

    if (poseOrjointFinal[15] == 1)
    {
        /* 如果给定的是位姿 先求出位姿对应的关节角  静态大范围求解模式 β扫描间隔 0.01*/
        InverseKinematics(angleInit, poseOrjointFinal, -M_PI, 0.01, M_PI, angleFianl_beta, angleFianl_beta_size);
    }
    else
    {
        for (i = 0; i < 7; i++)
        {
            angleFianl_beta[i] = poseOrjointFinal[i];
        }
    }

    /* 1x8:7个关节角+此时的β */
    /* 初始化 */
    c_Tmax = 0.0;

    /* workMode = 0：此时在关节空间对7个关节进行规划 */
    /* 每一个关节规划包含16个参数：[Ta, Tv, Td, Tj1, Tj2, q_0, q_1, v_0, v_1, vlim, a_max, a_min, a_lima, a_limd, j_max, j_min, signOfQ0Q1] */
    double limit[4] = {M_PI, M_PI, M_PI, speedRate};
    for (i = 0; i < 7; i++)
    {
        STrajectoryPara(angleInit[i], angleFianl_beta[i], limit, arm.motor[i].sp.para);

        t = arm.motor[i].sp.para[0] + arm.motor[i].sp.para[1] + arm.motor[i].sp.para[2];
        T[i] = t;
        if (c_Tmax < t)
        {
            c_Tmax = t;
        }
        arm.motor[i].sp.time = 0.0;
    }

    double s_planTimes = ceil(c_Tmax / 0.01); // 向上取整

    // ==================== 下面的代码为仿真，需要写在循环中 ====================== //
    printf("Start MoveJ Simulation\n");
    for (loop_ub = 0; loop_ub <= (int)s_planTimes; loop_ub++)
    {
        for (i = 0; i < 7; i++)
        {
            if (loop_ub < (int)s_planTimes)
            {
                arm.motor[i].exp_position = S_position(arm.motor[i].sp.time, arm.motor[i].sp.para);
                arm.motor[i].sp.time += 0.01 * T[i] / c_Tmax;
            }
            else // 最后一个
            {
                arm.motor[i].exp_position = angleFianl_beta[i];
            }
        }
        printf("%d,%d\n", loop_ub, arm.motor[6].exp_position);
    }
}

/*
 * 功能--------------实现机械臂末端直线插补姿态保持不变
 * 输入--------------angleInit：本时刻各关节角1x7
 *                   locationFinal：最终位置坐标1x3
 *                   speedRate：机械臂最大速度比例，取值0~1，用于调节机械臂最大速度
 * 输出--------------angleByPlanningL：返回多组离散关节角，每0.01s发送给驱动器一组
 */
void moveLPoseUnchanged(double angleInit[7], const double locationFinal[3], double speedRate, bodypart arm)
{
    double poseInit[16];
    double betaInit;
    double betaFinal;
    int i21;
    double b_poseInit[16];
    int idx;
    int poseInit_tmp;
    double angleFinal_data[8];
    int angleFinal_size[2];
    int i;
    boolean_T exitg1;
    double tmp_data[8];
    double y[7];
    boolean_T exitg2;
    double maxAngle;
    ForwardKinematics(angleInit, poseInit);

    /* 求解初末β */
    betaInit = FindBeta(angleInit);
    for (i21 = 0; i21 < 3; i21++)
    {
        idx = i21 << 2;
        b_poseInit[idx] = poseInit[idx];
        poseInit_tmp = 1 + idx;
        b_poseInit[poseInit_tmp] = poseInit[poseInit_tmp];
        idx += 2;
        b_poseInit[idx] = poseInit[idx];
        b_poseInit[12 + i21] = locationFinal[i21];
    }
    b_poseInit[3] = 0.0;
    b_poseInit[7] = 0.0;
    b_poseInit[11] = 0.0;
    b_poseInit[15] = 1.0;

    double locationInit[3];
    double locationDelta[3];
    memcpy(locationInit, *(double(*)[3]) & poseInit[12], sizeof(locationInit));
    locationDelta[0] = locationInit[0] - locationFinal[0];
    locationDelta[1] = locationInit[1] - locationFinal[1];
    locationDelta[2] = locationInit[2] - locationFinal[2];
    double distance = b_norm(locationDelta);

    InverseKinematics(angleInit, b_poseInit, -M_PI, 0.01, M_PI, angleFinal_data, angleFinal_size);
    betaFinal = angleFinal_data[7];
    /* 返回1x8矩阵，最后一个为beta值 */
    /* 对β进行S曲线规划,并求出旋转过程所需时间Tmax */

    double limit_line[4] = {1000.0 / distance, 1000.0 / distance, 1000.0 / distance, speedRate}; // 速度，加速度，加加速度的数值限制都为1000.0mm
    STrajectoryPara(0, 1, limit_line, arm.s_line.para);

    double limit_beta[4] = {1.0, 1.0, 1.0, 1.0};
    STrajectoryPara(betaInit, betaFinal, limit_beta, arm.s_beta.para);

    double T[2];
    T[0] = arm.s_line.para[0] + arm.s_line.para[1] + arm.s_line.para[2];
    T[1] = arm.s_beta.para[0] + arm.s_beta.para[1] + arm.s_beta.para[2];

    double T_max = max(T, 2);
    double s_planTimes = ceil(T_max / 0.01); // 向上取整
    arm.s_line.time = 0.0;
    arm.s_beta.time = 0.0;

    // ==================== 下面的代码为仿真，需要写在循环中 ====================== //
    printf("Start MoveL Simulation\n");
    double location[3];
    double t_line;
    double beta_line;
    double angle_planned[8];
    int angle_planned_size[2];
    double angle_delta[7];
    for (int loop = 0; loop <= (int)s_planTimes; loop++)
    {
        t_line = S_position(arm.s_line.time, arm.s_line.para);
        location[0] = locationInit[0] + (locationFinal[0] - locationInit[0]) * t_line;
        location[1] = locationInit[1] + (locationFinal[1] - locationInit[1]) * t_line;
        location[2] = locationInit[2] + (locationFinal[2] - locationInit[2]) * t_line;

        beta_line = S_position(arm.s_beta.time, arm.s_beta.para);

        // 将当前位置与初始姿态组成位姿矩阵
        for (i21 = 0; i21 < 3; i21++)
        {
            idx = i21 << 2;
            b_poseInit[idx] = poseInit[idx];
            poseInit_tmp = 1 + idx;
            b_poseInit[poseInit_tmp] = poseInit[poseInit_tmp];
            idx += 2;
            b_poseInit[idx] = poseInit[idx];
            b_poseInit[12 + i21] = location[i21];
        }
        b_poseInit[3] = 0.0;
        b_poseInit[7] = 0.0;
        b_poseInit[11] = 0.0;
        b_poseInit[15] = 1.0;

        InverseKinematics(angleInit, b_poseInit, beta_line, 0, beta_line, angle_planned, angle_planned_size);
        for (int j = 0; j < 7; j++)
        {
            arm.motor[j].exp_position = angle_planned[j];
            angle_delta[j] = fabs(angle_planned[j] - angleInit[j]);
        }
        maxAngle = max(angle_delta, 7);
        if (maxAngle > 0.05)
        {
            printf("failed when excuting\n");
        }

        arm.s_line.time += 0.01 * T[0] / T_max;
        arm.s_beta.time += 0.01 * T[1] / T_max;
        memcpy(angleInit, angle_planned, sizeof(angle_planned));

        printf("%d,%d\n", loop, arm.motor[6].exp_position);
    }
}

/*
 * 功能--------------实现机械臂末端直线插补姿态均匀变化,效果很不好……
 * 输入--------------angleInit：本时刻各关节角1x7
 *                   poseFinal：最终位姿4x4
 *                   speedRate：机械臂最大速度比例，取值0~1，用于调节机械臂最大速度
 * 输出--------------angleByPlanningL：返回多组离散关节角，每0.01s发送给驱动器一组
 */
void moveLPoseChanged(double angleInit[7], const double poseFinal[16], double speedRate, bodypart arm)
{
    double poseInit[16];
    int r2;
    int r1;
    int rtemp;
    double RRelative[9];
    int r3;
    double maxval;
    double a21;
    double relativeRot[9];
    double betaInit;
    double betaFinal;
    double relativeRot_tmp;
    double b_relativeRot_tmp;
    double c_relativeRot_tmp;
    int d_relativeRot_tmp;
    int e_relativeRot_tmp;
    double rEquivalent[4];
    double angleFinal_data[8];
    int angleFinal_size[2];
    double para[16];
    double Tofpara[3];
    boolean_T exitg1;
    double b_resultOfSPlanning_angle[4];
    double tmp_data[8];
    double y[7];
    boolean_T exitg2;

    /* 初始化 */
    ForwardKinematics(angleInit, poseInit);
    for (r2 = 0; r2 < 3; r2++)
    {
        rtemp = r2 << 2;
        RRelative[3 * r2] = poseInit[rtemp];
        RRelative[1 + 3 * r2] = poseInit[1 + rtemp];
        RRelative[2 + 3 * r2] = poseInit[2 + rtemp];
    }

    r1 = 0;
    r2 = 1;
    r3 = 2;
    maxval = fabs(poseInit[0]);
    a21 = fabs(poseInit[1]);
    if (a21 > maxval)
    {
        maxval = a21;
        r1 = 1;
        r2 = 0;
    }

    if (fabs(poseInit[2]) > maxval)
    {
        r1 = 2;
        r2 = 1;
        r3 = 0;
    }

    RRelative[r2] = poseInit[r2] / poseInit[r1];
    RRelative[r3] /= RRelative[r1];
    RRelative[3 + r2] -= RRelative[r2] * RRelative[3 + r1];
    RRelative[3 + r3] -= RRelative[r3] * RRelative[3 + r1];
    RRelative[6 + r2] -= RRelative[r2] * RRelative[6 + r1];
    RRelative[6 + r3] -= RRelative[r3] * RRelative[6 + r1];
    if (fabs(RRelative[3 + r3]) > fabs(RRelative[3 + r2]))
    {
        rtemp = r2;
        r2 = r3;
        r3 = rtemp;
    }

    RRelative[3 + r3] /= RRelative[3 + r2];
    RRelative[6 + r3] -= RRelative[3 + r3] * RRelative[6 + r2];
    relativeRot[3 * r1] = poseFinal[0] / RRelative[r1];
    maxval = RRelative[3 + r1];
    relativeRot[3 * r2] = poseFinal[4] - relativeRot[3 * r1] * maxval;
    a21 = RRelative[6 + r1];
    relativeRot[3 * r3] = poseFinal[8] - relativeRot[3 * r1] * a21;
    betaInit = RRelative[3 + r2];
    relativeRot[3 * r2] /= betaInit;
    relativeRot_tmp = RRelative[6 + r2];
    relativeRot[3 * r3] -= relativeRot[3 * r2] * relativeRot_tmp;
    b_relativeRot_tmp = RRelative[6 + r3];
    relativeRot[3 * r3] /= b_relativeRot_tmp;
    c_relativeRot_tmp = RRelative[3 + r3];
    relativeRot[3 * r2] -= relativeRot[3 * r3] * c_relativeRot_tmp;
    relativeRot[3 * r1] -= relativeRot[3 * r3] * RRelative[r3];
    relativeRot[3 * r1] -= relativeRot[3 * r2] * RRelative[r2];
    d_relativeRot_tmp = 1 + 3 * r1;
    relativeRot[d_relativeRot_tmp] = poseFinal[1] / RRelative[r1];
    rtemp = 1 + 3 * r2;
    relativeRot[rtemp] = poseFinal[5] - relativeRot[d_relativeRot_tmp] * maxval;
    e_relativeRot_tmp = 1 + 3 * r3;
    relativeRot[e_relativeRot_tmp] = poseFinal[9] - relativeRot[d_relativeRot_tmp] * a21;
    relativeRot[rtemp] /= betaInit;
    relativeRot[e_relativeRot_tmp] -= relativeRot[rtemp] * relativeRot_tmp;
    relativeRot[e_relativeRot_tmp] /= b_relativeRot_tmp;
    relativeRot[rtemp] -= relativeRot[e_relativeRot_tmp] * c_relativeRot_tmp;
    relativeRot[d_relativeRot_tmp] -= relativeRot[e_relativeRot_tmp] *
                                      RRelative[r3];
    relativeRot[d_relativeRot_tmp] -= relativeRot[rtemp] * RRelative[r2];
    d_relativeRot_tmp = 2 + 3 * r1;
    relativeRot[d_relativeRot_tmp] = poseFinal[2] / RRelative[r1];
    rtemp = 2 + 3 * r2;
    relativeRot[rtemp] = poseFinal[6] - relativeRot[d_relativeRot_tmp] * maxval;
    e_relativeRot_tmp = 2 + 3 * r3;
    relativeRot[e_relativeRot_tmp] = poseFinal[10] - relativeRot[d_relativeRot_tmp] * a21;
    relativeRot[rtemp] /= betaInit;
    relativeRot[e_relativeRot_tmp] -= relativeRot[rtemp] * relativeRot_tmp;
    relativeRot[e_relativeRot_tmp] /= b_relativeRot_tmp;
    relativeRot[rtemp] -= relativeRot[e_relativeRot_tmp] * c_relativeRot_tmp;
    relativeRot[d_relativeRot_tmp] -= relativeRot[e_relativeRot_tmp] *
                                      RRelative[r3];
    relativeRot[d_relativeRot_tmp] -= relativeRot[rtemp] * RRelative[r2];

    /* 求相对旋转矩阵的四元数 */
    rot2quat(relativeRot, rEquivalent);

    /* [theta,rx,ry,rz] */
    /* 求解初末β */
    betaInit = FindBeta(angleInit);
    InverseKinematics(angleInit, poseFinal, -M_PI, 0.01, M_PI, angleFinal_data, angleFinal_size); /* 返回1x8矩阵，最后一个为beta值 */
    betaFinal = angleFinal_data[7];

    double locationInit[3];
    double locationDelta[3];
    double locationFinal[3];
    memcpy(locationInit, *(double(*)[3]) & poseInit[12], sizeof(locationInit));
    memcpy(locationFinal, *(double(*)[3]) & poseFinal[12], sizeof(locationFinal));
    locationDelta[0] = locationInit[0] - locationFinal[0];
    locationDelta[1] = locationInit[1] - locationFinal[1];
    locationDelta[2] = locationInit[2] - locationFinal[2];
    double distance = b_norm(locationDelta);

    /* 返回1x8矩阵，最后一个为beta值 */
    /* 对β进行S曲线规划,并求出旋转过程所需时间Tmax */
    double limit_line[4] = {1000.0 / distance, 1000.0 / distance, 1000.0 / distance, speedRate}; // 速度，加速度，加加速度的数值限制都为1000.0mm
    STrajectoryPara(0, 1, limit_line, arm.s_line.para);

    double limit_beta[4] = {1.0, 1.0, 1.0, speedRate};
    STrajectoryPara(betaInit, betaFinal, limit_beta, arm.s_beta.para);

    double limit_equat[4] = {1.0, 1.0, 1.0, speedRate};
    STrajectoryPara(0, rEquivalent[0], limit_equat, arm.s_equat.para);

    double T[3];
    T[0] = arm.s_line.para[0] + arm.s_line.para[1] + arm.s_line.para[2];
    T[1] = arm.s_beta.para[0] + arm.s_beta.para[1] + arm.s_beta.para[2];
    T[2] = arm.s_equat.para[0] + arm.s_equat.para[1] + arm.s_equat.para[2];
    double Tmax = max(T, 3);

    double s_planTimes = ceil(Tmax / 0.01);
    arm.s_line.time = 0.0;
    arm.s_beta.time = 0.0;
    arm.s_equat.time = 0.0;

    // ==================== 下面的代码为仿真，需要写在循环中 ====================== //
    printf("Start MoveL change Simulation\n");
    double location[3];
    double t_line;
    double beta_line;
    double equat_line[4];
    double angle_planned[8];
    int angle_planned_size[2];
    double angle_delta[7];
    double rot_[9];
    double pose_line[16];
    equat_line[1] = rEquivalent[1];
    equat_line[2] = rEquivalent[2];
    equat_line[3] = rEquivalent[3];
    double R_init[9];
    double R[9];
    for (r2 = 0; r2 < 3; r2++)
    {
        rtemp = r2 << 2;
        R_init[3 * r2] = poseInit[rtemp];
        R_init[1 + 3 * r2] = poseInit[1 + rtemp];
        R_init[2 + 3 * r2] = poseInit[2 + rtemp];
    }
    for (int loop = 0; loop <= (int)s_planTimes; loop++)
    {
        t_line = S_position(arm.s_line.time, arm.s_line.para);
        location[0] = locationInit[0] + (locationFinal[0] - locationInit[0]) * t_line;
        location[1] = locationInit[1] + (locationFinal[1] - locationInit[1]) * t_line;
        location[2] = locationInit[2] + (locationFinal[2] - locationInit[2]) * t_line;

        beta_line = S_position(arm.s_beta.time, arm.s_beta.para);
        equat_line[0] = S_position(arm.s_equat.time, arm.s_equat.para);

        // 将当前位置与初始姿态组成位姿矩阵

        quat2rot(equat_line, rot_);
        matrixMultiply(rot_, 3, 3, R_init, 3, 3, R);
        int idx = 0, poseInit_tmp;
        for (int i21 = 0; i21 < 3; i21++)
        {
            pose_line[i21 * 4 + 0] = R[i21 * 3 + 0];
            pose_line[i21 * 4 + 1] = R[i21 * 3 + 1];
            pose_line[i21 * 4 + 2] = R[i21 * 3 + 2];
            pose_line[i21 * 4 + 3] = 0.0;
            pose_line[i21 + 12] = location[i21];
        }
        pose_line[15] = 1.0;

        InverseKinematics(angleInit, pose_line, beta_line, 0, beta_line, angle_planned, angle_planned_size);
        for (int j = 0; j < 7; j++)
        {
            arm.motor[j].exp_position = angle_planned[j];
            angle_delta[j] = fabs(angle_planned[j] - angleInit[j]);
        }
        double maxAngle = max(angle_delta, 7);
        if (maxAngle > 0.05)
        {
            printf("failed when excuting,%f\n", maxAngle);
        }

        arm.s_line.time += 0.01 * T[0] / Tmax;
        arm.s_beta.time += 0.01 * T[1] / Tmax;
        arm.s_equat.time += 0.01 * T[2] / Tmax;
        memcpy(angleInit, angle_planned, sizeof(angle_planned));
        printf("%d,%d\n", loop, arm.motor[0].exp_position);
    }
}

/*
 * TCP姿态可控/不变圆弧插补（笛卡尔空间）3
 * 功能-------------TCP姿态不变圆弧插补
 * 输入-------------angleInit：初始关节角1x7,可计算TCP初始点坐标
 *                  pointMiddle：TCP中间点坐标1x3
 *                  pointFinal：TCP最终点坐标1x3
 *                  speedRate：机械臂最大速度比例，取值0~1，用于调节机械臂最大速度
 * 输出-------------angleByPlanningC：返回多组离散关节角，每0.01s发送给驱动器一组
 * Arguments    : const double angleInit[7]
 *                const double pointMiddle[3]
 *                const double pointFinal[3]
 *                double speedRate
 *                emxArray_real_T *angleByPlanningC
 * Return Type  : void
 */
void moveCPoseUnchanged(double angleInit[7], const double pointMiddle[3], const double pointFinal[3], double speedRate, bodypart arm)
{
    double poseInit[16];
    double unusedU0_data[3];
    int unusedU0_size[2];
    double rad;
    double normalVector_data[3];
    int normalVector_size[2];
    double maxAngle;
    double T_data[16];
    int T_size[2];
    double theta13_data[4];
    double relativeRotationMatrix[9];
    double b_relativeRotationMatrix[9];
    double betaInit;
    int i;
    int aoffset;
    int m;
    double c_relativeRotationMatrix[9];
    int inner;
    double angleFinal_data[8];
    double dv3[2];
    double theta13[2];
    int b_i;
    boolean_T exitg1;
    double b_b[4];
    double location0_data[4];
    int k;
    double y[7];
    boolean_T exitg2;
    ForwardKinematics(angleInit, poseInit);

    /* 初始位姿4x4 */
    /* TCP初始点坐标 */
    ThreePointCircle(*(double(*)[3]) & poseInit[12], pointMiddle, pointFinal,
                     unusedU0_data, unusedU0_size, &rad, normalVector_data,
                     normalVector_size, &maxAngle, T_data, T_size);

    /* 等效旋转角 */
    /* 等效旋转轴1x3 */
    /* 四元数 */
    theta13_data[0] = maxAngle;
    theta13_data[1] = normalVector_data[0];
    theta13_data[2] = normalVector_data[1];
    theta13_data[3] = normalVector_data[2];

    quat2rot(theta13_data, relativeRotationMatrix);

    /* 相对旋转矩阵3x3 */
    /* 求解初末β  找出初始关节角对应β */
    betaInit = FindBeta(angleInit);

    double rot[9], pos[3];
    rotfromT(poseInit, rot);
    posfromT(poseInit, pos);

    double temp[9];
    matrixMultiply(relativeRotationMatrix, 3, 3, rot, 3, 3, temp);
    double poseFinal[16];
    TfromRotPos(temp, pointFinal, poseFinal);

    InverseKinematics(angleInit, poseFinal, -M_PI, 0.01, M_PI, angleFinal_data, unusedU0_size);

    /* 返回1x8矩阵，最后一个为beta值 */
    /* 同时对等效旋转角和β规划betaByPlanningJ */

    double limit_beta[4] = {1.0, 1.0, 1.0, speedRate};
    STrajectoryPara(betaInit, angleFinal_data[7], limit_beta, arm.s_beta.para);

    double limit_equat[4] = {1.0, 1.0, 1.0, speedRate};
    STrajectoryPara(0.0, maxAngle, limit_equat, arm.s_equat.para);

    double T[2];
    T[0] = arm.s_beta.para[0] + arm.s_beta.para[1] + arm.s_beta.para[2];
    T[1] = arm.s_equat.para[0] + arm.s_equat.para[1] + arm.s_equat.para[2];
    double Tmax = max(T, 2);

    double s_planTimes = ceil(Tmax / 0.01);
    arm.s_beta.time = 0.0;
    arm.s_equat.time = 0.0;

    // ==================== 下面的代码为仿真，需要写在循环中 ====================== //
    printf("Start MoveL change Simulation\n");
    double beta_line;
    double t_line;
    double equat_line[4];
    double angle_planned[8];
    int angle_planned_size[2];
    double angle_delta[7];
    double pose_line[16];

    for (int loop = 0; loop <= s_planTimes; loop++)
    {
        t_line = S_position(arm.s_equat.time, arm.s_equat.para);
        equat_line[0] = rad * cos(t_line);
        equat_line[1] = rad * sin(t_line);
        equat_line[2] = 0.0;
        equat_line[3] = 1.0;

        if (T_size[1] == 1)
        {
            m = T_size[0];
            for (i = 0; i < m; i++)
            {
                location0_data[i] = 0.0;
                for (aoffset = 0; aoffset < 1; aoffset++)
                {
                    location0_data[i] += T_data[i] * equat_line[0];
                }
            }
        }
        else
        {
            matrixMultiply(T_data, 4, 4, equat_line, 4, 1, location0_data);
        }

        /* 此时location0为4x1 */
        TfromRotPos(rot, *(double(*)[3]) & location0_data[0], pose_line);

        beta_line = S_position(arm.s_beta.time, arm.s_beta.para);
        InverseKinematics(angleInit, pose_line, beta_line, 0, beta_line, angle_planned, angle_planned_size);

        for (int j = 0; j < 7; j++)
        {
            arm.motor[j].exp_position = angle_planned[j];
            angle_delta[j] = fabs(angle_planned[j] - angleInit[j]);
        }
        double maxAngle = max(angle_delta, 7);
        if (maxAngle > 0.05)
        {
            printf("failed when excuting,%f\n", maxAngle);
        }

        arm.s_beta.time += 0.01 * T[0] / Tmax;
        arm.s_equat.time += 0.01 * T[1] / Tmax;
        memcpy(angleInit, angle_planned, sizeof(angle_planned));
        printf("%d,%d\n", loop, arm.motor[0].exp_position);
    }
}

/*
 * TCP姿态可控/不变圆弧插补（笛卡尔空间）2
 * 功能-------------TCP位姿均匀变化圆弧插补
 * 输入-------------angleInit：初始关节角1x7,可计算TCP初始点坐标
 *                  pointMiddle：TCP中间点坐标1x3
 *                  pointFinal：TCP最终点坐标1x3
 *                  speedRate：机械臂最大速度比例，取值0~1，用于调节机械臂最大速度
 * 输出-------------angleByPlanningC：返回多组离散关节角，每0.01s发送给驱动器一组，返回-1说明规划失败
 * Arguments    : const double angleInit[7]
 *                const double pointMiddle[3]
 *                const double pointFinal[3]
 *                double speedRate
 *                emxArray_real_T *angleByPlanningC
 * Return Type  : void
 */
void moveCPoseChanged(double angleInit[7], const double pointMiddle[3], const double pointFinal[3], double speedRate, bodypart arm)
{
    double poseInit[16];
    double unusedU0_data[3];
    int unusedU0_size[2];
    double rad;
    double normalVector_data[3];
    int normalVector_size[2];
    double maxAngle;
    double T_data[16];
    int T_size[2];
    double location0_data[4];
    double relativeRotationMatrix[9];
    double betaInit;
    int i, m, aoffset;
    double angleFinal_data[8];
    ForwardKinematics(angleInit, poseInit);

    /* 初始位姿4x4 */
    /* TCP初始点坐标 */
    ThreePointCircle(*(double(*)[3]) & poseInit[12], pointMiddle, pointFinal,
                     unusedU0_data, unusedU0_size, &rad, normalVector_data,
                     normalVector_size, &maxAngle, T_data, T_size);

    /* 等效旋转角 */
    /* 等效旋转轴1x3 */
    /* 四元数 */
    location0_data[0] = maxAngle;
    location0_data[1] = normalVector_data[0];
    location0_data[2] = normalVector_data[1];
    location0_data[3] = normalVector_data[2];

    quat2rot(location0_data, relativeRotationMatrix);

    /* 相对旋转矩阵3x3 */
    /* 求解初末β 找出初始关节角对应β */
    betaInit = FindBeta(angleInit);

    double rot[9], pos[3];
    rotfromT(poseInit, rot);
    posfromT(poseInit, pos);

    double temp[9];double poseFinal[16];
    matrixMultiply(relativeRotationMatrix, 3, 3, rot, 3, 3, temp);      //相对旋转矩阵乘初始旋转矩阵，得到末状态旋转矩阵
    TfromRotPos(temp, pointFinal, poseFinal);                           // 末状态旋转矩阵和位置，组成位姿矩阵

    InverseKinematics(angleInit, poseFinal, -M_PI, 0.01, M_PI, angleFinal_data, unusedU0_size); //计算逆运动学

    /* 返回1x8矩阵，最后一个为beta值 */
    /* 同时对等效旋转角和β规划betaByPlanningJ */

    double limit_beta[4] = {1.0, 1.0, 1.0, speedRate};
    STrajectoryPara(betaInit, angleFinal_data[7], limit_beta, arm.s_beta.para);

    double limit_equat[4] = {1.0, 1.0, 1.0, speedRate};
    STrajectoryPara(0.0, maxAngle, limit_equat, arm.s_equat.para);

    double T[2];
    T[0] = arm.s_beta.para[0] + arm.s_beta.para[1] + arm.s_beta.para[2];
    T[1] = arm.s_equat.para[0] + arm.s_equat.para[1] + arm.s_equat.para[2];
    double Tmax = max(T, 2);

    double s_planTimes = ceil(Tmax / 0.01);
    arm.s_beta.time = 0.0;
    arm.s_equat.time = 0.0;

// ==================== 下面的代码为仿真，需要写在循环中 ====================== //
    printf("Start MoveL change Simulation\n");
    double beta_line;
    double t_line;
    double rot_line[4];
    double equat_line[4];
    double angle_planned[8];
    int angle_planned_size[2];
    double angle_delta[7];
    double pose_line[16];
    double rot_[9];
    equat_line[1] = normalVector_data[0];
    equat_line[2] = normalVector_data[1];
    equat_line[3] = normalVector_data[2];

    for (int loop = 0; loop <= s_planTimes; loop++)
    {
        t_line = S_position(arm.s_equat.time, arm.s_equat.para);
        rot_line[0] = rad * cos(t_line);
        rot_line[1] = rad * sin(t_line);
        rot_line[2] = 0.0;
        rot_line[3] = 1.0;

        if (T_size[1] == 1)
        {
            m = T_size[0];
            for (i = 0; i < m; i++)
            {
                location0_data[i] = 0.0;
                for (aoffset = 0; aoffset < 1; aoffset++)
                {
                    location0_data[i] += T_data[i] * rot_line[0];
                }
            }
        }
        else
        {
            matrixMultiply(T_data, 4, 4, rot_line, 4, 1, location0_data);
        }

        /* 此时location0为4x1 */
        equat_line[0] = t_line;
        quat2rot(equat_line, rot_);
        double rot_temp[9];
        matrixMultiply(rot_, 3, 3, rot, 3, 3, rot_temp);
        TfromRotPos(rot_temp, *(double(*)[3]) & location0_data[0], pose_line);

        beta_line = S_position(arm.s_beta.time, arm.s_beta.para);
        InverseKinematics(angleInit, pose_line, beta_line, 0, beta_line, angle_planned, angle_planned_size);

        for (int j = 0; j < 7; j++)
        {
            arm.motor[j].exp_position = angle_planned[j];
            angle_delta[j] = fabs(angle_planned[j] - angleInit[j]);
        }
        double maxAngle = max(angle_delta, 7);
        if (maxAngle > 0.05)
        {
            printf("failed when excuting,%f\n", maxAngle);
        }

        arm.s_beta.time += 0.01 * T[0] / Tmax;
        arm.s_equat.time += 0.01 * T[1] / Tmax;
        memcpy(angleInit, angle_planned, sizeof(angle_planned));
        printf("%d,%d\n", loop, arm.motor[0].exp_position);
    }

}
