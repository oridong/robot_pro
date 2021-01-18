#include <math.h>
#include <stdio.h>
#include <string.h>
#include "math_tool.h"
#include "kinematics.h"
#include "S_trajectory.h"
#include "ThreePointCircle.h"
#include "control.h"
#include "rt_type/tmwtypes.h"

# define M_PI 3.14159265358979323846

/*
 * 功能： 大范围关节角转动（关节空间）
 * 输入-------------angleInit：初始关节角1x7
 *                  poseOrjointFinal：最终位姿4x4，需要先初始化为零
 *                  speedRate：机械臂最大速度比例，取值0~1，用于调节机械臂最大速度
 * 输出-------------arm.splan：返回规划参数，每0.01s发送给驱动器一组
 */
void moveJ(bodypart &arm, double jointFinal[7], double speedRate)
{
    double angleFianl_beta[8];
    int angleFianl_beta_size[2];
    double c_Tmax = 0.0;
    int i;
    double angleInit[7];
    double T[7];
    int motornum = arm.motornum; 
    for (i = 0; i < motornum; i++)
    {
        angleInit[i] = arm.motor[i].ref_position/arm.jointGear[i];
    }
    double sum = 0.0;
            
    /* workMode = 0：此时在关节空间对7个关节进行规划 */
    /* 每一个关节规划包含16个参数：[Ta, Tv, Td, Tj1, Tj2, q_0, q_1, v_0, v_1, vlim, a_max, a_min, a_lima, a_limd, j_max, j_min, signOfQ0Q1] */
    double limit[4] = {M_PI/4, M_PI, M_PI, speedRate};

    // 检查是否在目标位置，如果在直接返回，但是由于电机抖动可能难以进入，需要提高s曲线规划兼容性
    for (i = 0; i < motornum; i++)
    {
        sum += fabs(angleInit[i] - jointFinal[i]);
    }
    if (sum == 0.0)
    {
        printf("same goal\n");
        arm.state = IDLE;
        return;
    }
    
    // 遍历电机进行s曲线规划
    for (i = 0; i < motornum; i++)
    {
        STrajectoryPara(angleInit[i], jointFinal[i], limit, arm.motor[i].sp.para);

        T[i] = arm.motor[i].sp.para[0] + arm.motor[i].sp.para[1] + arm.motor[i].sp.para[2];
        
        if (c_Tmax < T[i])
        {
            c_Tmax = T[i];
        }
        arm.motor[i].sp.time = 0.0;
    }

    // 遍历电机赋值规划增量时间
    for (i = 0; i < motornum; i++)
    {
        if (c_Tmax != 0.0){
            arm.motor[i].sp.deltaTime = arm.itp_period_times * double(ctl_period) / 1e9 * T[i] / c_Tmax;
        }
        // if (i == 1) printf("%f,%f\n",arm.jointPos[1],jointFinal[1]);
    }

    // 改变机械臂状态，并得到规划总次数
    arm.s_planTimes = (int)(ceil(c_Tmax / ((double)arm.itp_period_times * double(ctl_period) / 1e9))); // 向上取整 定为0.01
    arm.state = ON_MOVEJ;
    
    arm.plan_cnt = 0;
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

        printf("%d,%f\n", loop, arm.motor[6].exp_position);
    }
}

/*
 * 功能--------------实现机械臂末端直线插补姿态均匀变化,效果很不好……
 * 输入--------------angleInit：本时刻各关节角1x7
 *                   poseFinal：最终位姿 欧拉角，zyx顺规
 *                   speedRate：机械臂最大速度比例，取值0~1，用于调节机械臂最大速度
 * 输出--------------angleByPlanningL：返回多组离散关节角，每0.01s发送给驱动器一组
 */
void moveLPoseChanged(bodypart &arm, double Tfinal[16], double speedRate)
{
    double RRelative[9];
    double relativeRot[9];
    double betaInit;
    double betaFinal;
    double angleFinal_data[8];
    int angleFinal_size[2];
    double poseInit[16];

    // TfromPose(poseFinal, Tfinal);

    double rotInit[9],rotFinal[9],rotInit_t[9];
    /* 初始化 */
    ForwardKinematics(arm.jointPos, poseInit);

    rotfromT(poseInit, rotInit);
    rotfromT(Tfinal, rotFinal);
    matrixTrans(rotInit, 3, 3, rotInit_t);
    matrixMultiply(rotFinal, 3, 3, rotInit_t,3, 3, relativeRot);
    
    memcpy(arm.rotInit, rotInit, sizeof(rotInit));
    /* 求相对旋转矩阵的四元数 */
    rot2quat(relativeRot, arm.rEquivalent);

    /* [theta,rx,ry,rz] */
    /* 求解初末β */
    betaInit = FindBeta(arm.jointPos);

    printf_d(arm.jointPos,7);
    printf_d(Tfinal, 16);
    printf("betainit: %f\n",betaInit);
    InverseKinematics(arm.jointPos, Tfinal, betaInit-0.2, 0.01, betaInit + 0.2, angleFinal_data, angleFinal_size); /* 返回1x8矩阵，最后一个为beta值 */
    betaFinal = angleFinal_data[7];

    printf("betafinal: %f\n",betaFinal);
    printf_d(angleFinal_data,8);

    double locationFinal[3];
    memcpy(arm.locationInit, *(double(*)[3]) & poseInit[12], sizeof(arm.locationInit));
    memcpy(locationFinal, *(double(*)[3]) & Tfinal[12], sizeof(locationFinal));
    arm.locationDelta[0] = -arm.locationInit[0] + locationFinal[0];
    arm.locationDelta[1] = -arm.locationInit[1] + locationFinal[1];
    arm.locationDelta[2] = -arm.locationInit[2] + locationFinal[2];
    double distance = b_norm(arm.locationDelta);
    printf("deltalocation:%f\n", distance);

    /* 返回1x8矩阵，最后一个为beta值 */
    /* 对β进行S曲线规划,并求出旋转过程所需时间Tmax */
    double limit_line[4] = {1000.0 / distance, 1000.0 / distance, 1000.0 / distance, speedRate}; // 速度，加速度，加加速度的数值限制都为1000.0mm
    STrajectoryPara(0, 1, limit_line, arm.s_line.para);

    double limit_beta[4] = {1.0, 1.0, 1.0, speedRate};
    STrajectoryPara(betaInit, betaFinal, limit_beta, arm.s_beta.para);

    double limit_equat[4] = {1.0, 1.0, 1.0, speedRate};
    STrajectoryPara(0, arm.rEquivalent[0], limit_equat, arm.s_equat.para);

    double T[3];
    T[0] = arm.s_line.para[0] + arm.s_line.para[1] + arm.s_line.para[2];
    T[1] = arm.s_beta.para[0] + arm.s_beta.para[1] + arm.s_beta.para[2];
    T[2] = arm.s_equat.para[0] + arm.s_equat.para[1] + arm.s_equat.para[2];
    double Tmax = max(T, 3);

    arm.s_planTimes = (int)(ceil(Tmax / 0.01));     // 暂定为0.01,如果修改需要对应
    arm.s_line.time = 0.0;
    arm.s_beta.time = 0.0;
    arm.s_equat.time = 0.0;

    arm.s_line.deltaTime = 0.01 * T[0] / Tmax;
    arm.s_beta.deltaTime = 0.01 * T[1] / Tmax;
    arm.s_equat.deltaTime = 0.01 * T[2] / Tmax;

    arm.state = ON_MOVEL;
    arm.plan_cnt = 0;
    arm.itp_period_times = 10;
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
        printf("%d,%f\n", loop, arm.motor[0].exp_position);
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
        printf("%d,%f\n", loop, arm.motor[0].exp_position);
    }

}

int forceUpdate(bodypart &arm, int type, double dt, int dir_enable[6])
{
    double xdd[6];
    int i;

    double angleRef[7];
    double angleNow[7];
    double angleExp[8];
    int angleExpsize[2];
    double deltatrans[6];
    double ft[6];

    double dtrans[16];
    double rot_[9];
    double Tref[16];
    double beta;
    double temp[16];
    double pose_[6];

    // case 2
    double invJ[42];
    double jointp[7];
    double jointv[7];

    double normOfPose = 0.0;

    double force_vlim[6] = {50.0, 50.0, 80.0, 7.0, 7.0, 7.0}; 
    double force_flim[6] = {200.0, 200.0, 200.0, 50.0, 50.0, 50.0}; 

    extern FILE *fp;

    for ( i = 0; i < arm.motornum; i++)
    {
        angleRef[i] = arm.motor[i].ref_position / arm.jointGear[i];
        angleNow[i] = arm.jointPos[i];
    }

    // 进入力控退出模式，则切断力传感器开关
    for (i = 0; i< 6; i++)
    {
        if (arm.fctrl.Switch == 1)
        {
            ft[i] = arm.endft.ft[i];

            // 力超出限幅值，保护
            if (fabs(ft[i]) > force_flim[i])
            {
                arm.fctrl.Switch = 2;
                printf("Out of Force Limit!! Stop it\n");
                return 0;
            }
        }
        else
        {
            ft[i] = 0.0;
        }
    }
    // printf_d(ft, 6);

    for (i = 0; i < 6 ; i++)
    {
        if (dir_enable[i] == 1)
        {
            xdd[i] = (ft[i] - arm.fctrl.paramC[i] * arm.fctrl.totalV[i] - arm.fctrl.paramK[i] * arm.fctrl.totalP[i])/arm.fctrl.paramM[i];
        }
        else
        {
            xdd[i] = 0.0;
        }
        
        arm.fctrl.totalP[i] += (0.5 * dt * dt * xdd[i] + arm.fctrl.totalV[i] * dt);
        arm.fctrl.totalV[i] += xdd[i] * dt;
        deltatrans[i] = arm.fctrl.totalV[i] * dt;
    }

    double rot[9] = {1.0, deltatrans[5], -deltatrans[4],  
                    -deltatrans[5], 1.0, deltatrans[3], 
                    deltatrans[4], -deltatrans[3], 1.0};  
    switch (type)
    {
        case 0:     // 最稳方式
            
            TfromPose(arm.fctrl.totalP, dtrans);
            
            dtrans[12] *= 1000;
            dtrans[13] *= 1000;
            dtrans[14] *= 1000;
            ForwardKinematics(angleRef, Tref);  

            matrixMultiply(Tref, 4, 4, dtrans, 4, 4, temp);
           // beta = FindBeta(angleNow);
 	    beta = arm.betaExp;
            InverseKinematics(angleNow, temp, beta, 0, beta, angleExp, angleExpsize);
            // fprintf(fp, "%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%f,%f,%f,%f,%f,%f\n", arm.fctrl.totalP[0], arm.fctrl.totalP[1], arm.fctrl.totalP[2], arm.fctrl.totalP[3], arm.fctrl.totalP[4], arm.fctrl.totalP[5],arm.endft.ft[0],arm.endft.ft[1],arm.endft.ft[2],arm.endft.ft[3],arm.endft.ft[4],arm.endft.ft[5]);
        break;

        case 1:

            schmdit(rot, rot_);
            TfromRotPos(rot_, *(double (*)[3])&deltatrans[0], dtrans);
            matrixMultiply(arm.fctrl.totalTrans, 4, 4, dtrans, 4, 4, temp);
            memcpy(arm.fctrl.totalTrans, temp, sizeof(temp));

            // 重新使用dtrans 进行单位转换
            memcpy(dtrans, arm.fctrl.totalTrans, sizeof(dtrans));
            dtrans[12] *= 1000;
            dtrans[13] *= 1000;
            dtrans[14] *= 1000;
            ForwardKinematics(angleRef, Tref);

            matrixMultiply(Tref, 4, 4, dtrans, 4, 4, temp);
            beta = FindBeta(angleNow);

            InverseKinematics(angleNow, temp, beta, 0.0, beta, angleExp, angleExpsize);

           
        break;

        case 2:
            
            if (InvJacobian(angleNow, invJ))
                angleExpsize[1] = 8;
            else
                angleExpsize[1] = 0;
            
            for (i = 0; i< 6; i++)
            {
                if (i < 3)
                {
                    deltatrans[i] = arm.fctrl.totalV[i] * 1000;
                }
                else
                {
                    deltatrans[i] = arm.fctrl.totalV[i];
                }

                // limit velocity 
                if (deltatrans[i] > force_vlim[i])
                {
                    deltatrans[i] = force_vlim[i];
                    printf("velocity limited\n");
                }
                else if (deltatrans[i] < -force_vlim[i])
                {
                    deltatrans[i] = -force_vlim[i];
                    printf("velocity limited\n");
                }
            }
            matrixMultiply(invJ, 7, 6, deltatrans, 6, 1, jointv);
            printf_d(jointv, 7);
            for ( i = 0; i< 7; i++)
            {
                arm.fctrl.jointP[i] += jointv[i] * dt;
                angleExp[i] = angleRef[i] + arm.fctrl.jointP[i];
            }

        break;
    }
    
    if (angleExpsize[1] == 8){
        for ( i = 0; i < arm.motornum; i++)
        {
            arm.motor[i].exp_position = angleExp[i] * arm.jointGear[i];
        }
    }
    else{
        printf("fctrl inverse kinematics failed\n");
    }

    normOfPose = norm(arm.fctrl.totalP, 6);
    if (normOfPose< 1e-3)
    {
        return 0;
    }

    return 1;
    
}
