
/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2009-2010  Moehwald GmbH B. Benner
 *                     2011  IgH Andreas Stewering-Bone
 *                     2012  Florian Pose <fp@igh-essen.com>
 *
 *  This file is part of the IgH EtherCAT master
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT master. If not, see <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

#include "control.h"
#include "communication.h"
#include "moveX.h"
#include "kinematics.h"
#include "S_trajectory.h"
#include "math_tool.h"


# define LEFT 0
# define RIGHT 1
# define HEAD 2
# define LEG 3
/******************* CanOpen控制字 *******************/
#define SM_trans2 0x0006
#define SM_trans3 0x0007
#define SM_trans4 0x000f
#define SM_trans5 0x0007
#define VoltageDisable 0x0000
#define QuickStop 0x0002
#define FaultReset 0x0080

/******************* CanOpen状态字 *******************/
#define OPERATION_ENABLE 0
#define SWITCH_ON_DISABLED 1
#define READY_TO_SWITCH_ON 2
#define SWITCHED_ON 3
#define FAULT 4
#define QUICK_STOP_ACTIVE 5

/******************* 电机运行状态 *******************/
#define CONFIG_ELMO 0
#define CONTROL 1
#define EXIT 2
#define ENABLE 3

#define PI 3.1415926
#define RAD2DEG 180.0 / PI
#define DEG2RAD PI / 180.0
#define ELMO_GOLD 0x0000009a, 0x00030924
#define ATI_FTSENSOR 0x00000732, 0x26483052

#define R1 40       // mm 单位 远端半径
#define R2 30       // mm 单位 近端半径
#define radius 10       // mm 单位

/***************************************************************
 * 变量及参数， 注意：电机参数列表需要大于使用电机数量
 /**************************************************************/

#define MotorNum 1 // 使用的电机数量

int leftarm_use_motor[8] = {1, 1, 1, 1, 0, 1, 1, 1};
int rightarm_use_motor[8] = {1, 1, 1, 1, 0, 1, 1, 0};
// EtherCAT 电机总线地址
static EC_position left_slave_pos[] = {{0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {0, 7}}; 
static EC_position right_slave_pos[] = {{0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {0, 7}};
static EC_position head_slave_pos[] = {{0, 0}, {0, 1}, {0, 2}};
static EC_position track_slave_pos[] = {{0, 0}, {0, 1}, {0, 2}, {0, 2}, {0, 2}, {0, 2}, {0, 2}, {0, 0}, {0, 0}};
  
static double leftarmGear[7] = {160.0 * 20480 * 2.0 / PI, 160.0 * 20480 * 2.0 / PI, 100.0 * 20480 * 2.0 / PI, 100.0 * 20480 * 2.0 / PI, 100.0 * 18000 * 2.0 / PI, 200 * 32768 * 2.0 / PI, 200 * 32768 * 2.0 / PI};
static double rightarmGear[7] = {160.0 * 20480 * 2.0 / PI, 160.0 * 20480 * 2.0 / PI, 100.0 * 20480 * 2.0 / PI, 100.0 * 20480 * 2.0 / PI, 100.0 * 18000 * 2.0 / PI, 200 * 32768 * 2.0 / PI, 200 * 32768 * 2.0 / PI};
static double headGear[3] = {100.0, 100.0, 100.0};
static double trackGear[9] = {10.0/radius/2.0/PI*524288, 10.0/radius/2.0/PI*524288, 10.0/radius/2.0/PI*524288, 10.0/radius/2.0/PI*524288, 100.0, 100.0, 100.0, 100.0, 100.0};

const static uint8_t armMotorMode = 8; // 机械臂电机运行模式
const static uint8_t headMotorMode = 8; // 头部电机运行模式
const static uint8_t trackSpeedMotorMode = 9; // 履带速度电机运行模式
const static uint8_t trackJointMotorMode = 8; // 履带关节电机运行模式

// 插值周期
const uint8_t armMotoritpTimes[] = {10, 10, 10, 10, 10, 10, 10}; // 在初始化时对电机设置，能够对不同电机进行不同的设置, 默认插值倍数（插值周期与控制周期比）为10
const uint8_t headMotoritpTimes[] = {10, 10, 10};
const uint8_t trackMotoritpTimes[] = {10, 10, 10, 10, 10, 10, 10, 10, 10};      // 4履带 + 1腰 + 4关节

const int leftoffsetAngle[7] = {0, 0, 0, 0, 0, 0, 0};        // 单位弧度
const int rightoffsetAngle[7] = {0, 0, 0, 0, 0, 0, 0};
const int headoffsetAngle[3] = {0, 0, 0};
const int legoffsetAngle[5] = {0, 0, 0, 0, 0};


// 变量声明
bodypart leftarm;
bodypart rightarm;
bodypart head;
bodypart leg;
trackpart track;

double alpha[itp_window] = {0.0f};
double alpha_sum = 0.0f;
RT_TASK my_task;

double defaultM[6] = {100, 100, 180, 5, 5, 5};
double defaultEP[6] = {1.1, 1.1, 1.1, 1.1, 1.1, 1.1};				/*阻尼比*/
double defaultK[6] = {150, 150, 200, 3, 3, 3};			/*刚度A6D_wn.^2.*A6D_m*/


/************************************************
 * 重要状态位
 * *********************************************/
static int run = 1;
static int main_exit = 0;

/************************************************
 * EtherCat 相关结构体
 * *********************************************/
static ec_master_t *master0 = NULL;
static ec_master_state_t master0_state = {};
static ec_master_t *master1 = NULL;
static ec_master_state_t master1_state = {};

EC_domain domain[5];

// position mode 
ec_pdo_entry_info_t spe_pos[] = {  /* Slave Pdo Entries*/
    {0x607A, 0x00, 32}, /* Target position */
    {0x60fe, 0x01, 32}, /* Physical outputs */
    {0x6040, 0x00, 16}, /* Controlword */
    {0x6060, 0x00, 8},  /* mode of operation */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x60fd, 0x00, 32}, /* Digital inputs */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x2203, 0x00, 32}  /* Analog Input*/
};

ec_pdo_info_t sp_pos[] = {  /* slave_pdos */
    {0x1600, 3, spe_pos + 0}, /* RPDO1 Mapping */
    {0x160B, 1, spe_pos + 3}, /* RPDO1 Mapping */
    {0x1a00, 3, spe_pos + 4}, /* TPDO1 Mapping */
    {0x1A24, 1, spe_pos + 7}  /* TPDO1 Mapping */
};

ec_sync_info_t ss_pos[] = { /* slave_sync */ 
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, sp_pos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, sp_pos + 2, EC_WD_DISABLE},
    {0xff}
};

// velocity mode
ec_pdo_entry_info_t spe_vel[] = {  /* Slave Pdo Entries*/
    {0x60ff, 0x00, 32}, /* Target velocity */
    {0x6040, 0x00, 16}, /* Controlword */
    {0x6060, 0x00, 8},  /* mode of operation */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x60fd, 0x00, 32}, /* Digital inputs */
    {0x606c, 0x00, 32}, /* Velocity actual value */
    {0x6041, 0x00, 16}, /* Statusword */
};

ec_pdo_info_t sp_vel[] = {  /* slave_pdos */
    {0x1601, 2, spe_vel + 0}, /* RPDO1 Mapping */
    {0x160B, 1, spe_vel + 2}, /* RPDO1 Mapping */
    {0x1a03, 4, spe_vel + 3}, /* TPDO1 Mapping */
};

ec_sync_info_t ss_vels[] = { /* slave_sync */ 
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, sp_vel + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, sp_vel + 2, EC_WD_DISABLE},
    {0xff}
};

/*
 * Ethercat 检查函数
 */
void rt_check_domain_state(ec_domain_t *a, ec_domain_state_t result)
{
    ec_domain_state_t ds = {};

    ecrt_domain_state(a, &ds);

    if (ds.working_counter != result.working_counter)
    {
        rt_printf("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != result.wc_state)
    {
        rt_printf("Domain1: State %u.\n", ds.wc_state);
    }

    result = ds;
}

void rt_check_master_state(ec_master_t *m, ec_master_state_t result)
{
    ec_master_state_t ms;

    ecrt_master_state(m, &ms);

    if (ms.slaves_responding != result.slaves_responding)
    {
        rt_printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != result.al_states)
    {
        rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != result.link_up)
    {
        rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    result = ms;
}

void nsleep(long nano)
{
    struct timespec sleeptime;
    sleeptime.tv_nsec = nano;
    sleeptime.tv_sec = 0;
    nanosleep(&sleeptime, NULL);
}

/*
 * 初始化插值位置滤波器参数
 */
void ip_param_init()
{
    printf("位置滤波器参数：");
    for (int j = 0; j < itp_window; j++)
    {
        //用钟形分段直线来计算权重系数
        if (j < int((itp_window) / 2))
        {
            alpha[j] = 1 + 3 * j;
        }
        else if (j > int((itp_window) / 2))
        {
            alpha[j] = 1 + 3 * (itp_window - j - 1);
        }
        else
        {
            if ((itp_window) % 2 == 0)
            {
                alpha[j] = alpha[j - 1];
            }
            else
            {
                alpha[j] = 1 + 3 * j;
            }
        }
        alpha_sum += alpha[j];
        printf("%f\t", alpha[j]);
    }
    printf("\n");
}

/*
/* 初始化左臂电机，并将其设定为位置模式 
*  输入：记录结构体，主站序号，总线位置数组
**/
int leftarmInit(bodypart &arm, ec_master_t *m, int dm_index, EC_position * motor_pos)
{
    arm.dm_index = dm_index;        // 记录当前身体部分所使用的domain
    int i = 0,j = 0;
    arm.movefollowCnt = 0;
    arm.motornum = 7;
    arm.state = DISABLE;
    arm.fctrl.Switch = 0;

    uint8_t addr = 0;

    for (i = 0; i < arm.motornum; i++)     // ！！！！！！！！！ 只初始化一个电机作为实验 
    {
        arm.motor[i].first_time = 0;
        arm.motor[i].exp_position = 0;
        arm.motor[i].act_position = 0;
        arm.motor[i].start_pos = 0;
        arm.motor[i].mode = armMotorMode;
        arm.motor[i].this_send = 0;
        arm.motor[i].itp_period_times = armMotoritpTimes[i];
        arm.motor[i].plan_cnt = 0;
        arm.motor[i].plan_run_time = 0.0f;

        arm.jointGear[i] = leftarmGear[i];
        arm.offsetAngle[i] = leftoffsetAngle[i];
        arm.startJointAngle[i] = 0.0;
        arm.jointPos[i] = 0.0;
        arm.test_T = 5.0;
        arm.test_A = 90.0 * DEG2RAD;
        arm.motor_use[i] = leftarm_use_motor[i];

        if (arm.motor_use[i] == 1){
            arm.motor[i].alias = motor_pos[addr].alias;
            arm.motor[i].buspos = motor_pos[addr].buspos;
            addr ++ ;
            ec_pdo_entry_reg_t temp1 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[0].index, spe_pos[0].subindex, &arm.motor[i].offset.target_position, NULL};
            ec_pdo_entry_reg_t temp2 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[1].index, spe_pos[1].subindex, &arm.motor[i].offset.DO, NULL};
            ec_pdo_entry_reg_t temp3 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[2].index, spe_pos[2].subindex, &arm.motor[i].offset.ctrl_word, NULL};
            ec_pdo_entry_reg_t temp4 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[3].index, spe_pos[3].subindex, &arm.motor[i].offset.mode_operation, NULL};
            ec_pdo_entry_reg_t temp5 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[4].index, spe_pos[4].subindex, &arm.motor[i].offset.act_position, NULL};
            ec_pdo_entry_reg_t temp6 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[5].index, spe_pos[5].subindex, &arm.motor[i].offset.DI, NULL};
            ec_pdo_entry_reg_t temp7 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[6].index, spe_pos[6].subindex, &arm.motor[i].offset.status_word, NULL};
            ec_pdo_entry_reg_t temp8 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[7].index, spe_pos[7].subindex, &arm.motor[i].offset.ain, NULL};

            domain[dm_index].domain_reg.push_back(temp1);
            domain[dm_index].domain_reg.push_back(temp2);
            domain[dm_index].domain_reg.push_back(temp3);
            domain[dm_index].domain_reg.push_back(temp4);
            domain[dm_index].domain_reg.push_back(temp5);
            domain[dm_index].domain_reg.push_back(temp6);
            domain[dm_index].domain_reg.push_back(temp7);
            domain[dm_index].domain_reg.push_back(temp8);

            arm.motor[i].sc_dig_out = ecrt_master_slave_config(m, arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD);
            if (!arm.motor[i].sc_dig_out)
            {
                fprintf(stderr, "Failed to get slave configuration, No.%d.\n", i);
                return 0;
            }
            if (ecrt_slave_config_pdos(arm.motor[i].sc_dig_out, EC_END, ss_pos))
            {
                fprintf(stderr, "Failed to configure PDOs. No.%d.\n", i);
                return 0;
            }
       
            // ==================== 读写 SDO，配置0x2f41:0x00, 加入AD-input2到PDO中 ======================== //
            // 1、读入0x2f41:0x00 当前的值
            uint8_t result[4];
            size_t target_size = 4;
            size_t result_size;
            uint32_t abort_code;

            if (ecrt_master_sdo_upload(m, arm.motor[i].buspos, 0x2F41, 0, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
            {
                fprintf(stderr, "Failed to get sdo data.\n");
                return 0;
            }

            // 2、写入0x40000 到0x2f41:0x00 配置用户自定义 pdo 的功能
            uint32_t data = 0x40000;
            uint8_t *data_send = (uint8_t *)&data;
            size_t data_size = sizeof(data);
            if (ecrt_master_sdo_download(m, arm.motor[i].buspos, 0x2f41, 0, data_send, data_size, &abort_code)) // 写SDO，0x2F41:0 16-19位配置为4 即可开启模拟二通道输入
            {
                fprintf(stderr, "Failed to Set App to analog input 2.\n");
                return 0;
            }

            // 3、重读检验是否成功写入
            if (ecrt_master_sdo_upload(m, arm.motor[i].buspos, 0x2F41, 0, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
            {
                fprintf(stderr, "Failed to get sdo data.\n");
                return 0;
            }

            // =========================================================================== //

            if (ecrt_master_sdo_upload(m, arm.motor[i].buspos, 0x6064, 0, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
            {
                fprintf(stderr, "Failed to get sdo data.\n");
                return 0;
            }
            arm.motor[i].start_pos = *((uint32_t *)result);

            if (ecrt_master_sdo_upload(m, arm.motor[i].buspos, 0x3091, 2, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
            {
                fprintf(stderr, "Failed to get sdo data.\n");
                return 0;
            }
            arm.startJointAngle[i] = 2 * PI - *((int32_t *)result) / 131072.0 * PI;  // ()/262144 * 2 * -PI
            if (i < 4)
                arm.startJointAngle[i] = 0.0;
            // if (arm.startJointAngle[i] < -PI)
            // {
            //     arm.startJointAngle[i] += 2 * PI;
            // }
            // else if (arm.startJointAngle[i] >= PI)
            // {
            //     arm.startJointAngle[i] -= 2 * PI;
            // }
            printf("sdo config OK.%d,abs pos:%f rad,rel pos:%d\n", i, arm.startJointAngle[i], arm.motor[i].start_pos);
        }

    } /* motor 循环*/

    // 初始化力传感器从站
    if (arm.motor_use[7] == 1){
        EC_position ft_pos = {motor_pos[addr].alias, motor_pos[addr].buspos};
        i = FT_sensor_init(arm, m, dm_index, ft_pos);
    }
    
    return i;
    // return 1;
}

/*
/* 初始化右臂电机，并将其设定为位置模式 
*  输入：记录结构体，主站序号，总线位置数组
**/
int rightarmInit(bodypart &arm, ec_master_t *m, int dm_index, EC_position * motor_pos)
{

    arm.dm_index = dm_index;        // 记录当前身体部分所使用的domain
    int i = 0,j = 0;
    arm.movefollowCnt = 0;
    arm.motornum = 7;
    arm.state = DISABLE;
    arm.fctrl.Switch = 0;

    uint8_t addr = 0;

    for (i = 0; i < arm.motornum; i++)     // ！！！！！！！！！ 只初始化一个电机作为实验 
    {
        arm.motor[i].first_time = 0;
        arm.motor[i].exp_position = 0;
        arm.motor[i].act_position = 0;
        arm.motor[i].start_pos = 0;
        arm.motor[i].mode = armMotorMode;
        arm.motor[i].this_send = 0;
        arm.motor[i].itp_period_times = armMotoritpTimes[i];
        arm.motor[i].plan_cnt = 0;
        arm.motor[i].plan_run_time = 0.0f;

        arm.jointGear[i] = rightarmGear[i];
        arm.offsetAngle[i] = rightoffsetAngle[i];
        arm.startJointAngle[i] = 0.0;
        arm.jointPos[i] = 0.0;
        arm.test_T = 5.0;
        arm.test_A = 90.0 * DEG2RAD;
        arm.motor_use[i] = rightarm_use_motor[i];

        if (arm.motor_use[i] == 1){
            arm.motor[i].alias = motor_pos[addr].alias;
            arm.motor[i].buspos = motor_pos[addr].buspos;
            addr ++ ;
            ec_pdo_entry_reg_t temp1 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[0].index, spe_pos[0].subindex, &arm.motor[i].offset.target_position, NULL};
            ec_pdo_entry_reg_t temp2 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[1].index, spe_pos[1].subindex, &arm.motor[i].offset.DO, NULL};
            ec_pdo_entry_reg_t temp3 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[2].index, spe_pos[2].subindex, &arm.motor[i].offset.ctrl_word, NULL};
            ec_pdo_entry_reg_t temp4 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[3].index, spe_pos[3].subindex, &arm.motor[i].offset.mode_operation, NULL};
            ec_pdo_entry_reg_t temp5 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[4].index, spe_pos[4].subindex, &arm.motor[i].offset.act_position, NULL};
            ec_pdo_entry_reg_t temp6 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[5].index, spe_pos[5].subindex, &arm.motor[i].offset.DI, NULL};
            ec_pdo_entry_reg_t temp7 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[6].index, spe_pos[6].subindex, &arm.motor[i].offset.status_word, NULL};
            ec_pdo_entry_reg_t temp8 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[7].index, spe_pos[7].subindex, &arm.motor[i].offset.ain, NULL};

            domain[dm_index].domain_reg.push_back(temp1);
            domain[dm_index].domain_reg.push_back(temp2);
            domain[dm_index].domain_reg.push_back(temp3);
            domain[dm_index].domain_reg.push_back(temp4);
            domain[dm_index].domain_reg.push_back(temp5);
            domain[dm_index].domain_reg.push_back(temp6);
            domain[dm_index].domain_reg.push_back(temp7);
            domain[dm_index].domain_reg.push_back(temp8);

            arm.motor[i].sc_dig_out = ecrt_master_slave_config(m, arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD);
            if (!arm.motor[i].sc_dig_out)
            {
                fprintf(stderr, "Failed to get slave configuration, No.%d.\n", i);
                return 0;
            }
            if (ecrt_slave_config_pdos(arm.motor[i].sc_dig_out, EC_END, ss_pos))
            {
                fprintf(stderr, "Failed to configure PDOs. No.%d.\n", i);
                return 0;
            }
       
            // ==================== 读写 SDO，配置0x2f41:0x00, 加入AD-input2到PDO中 ======================== //
            // 1、读入0x2f41:0x00 当前的值
            uint8_t result[4];
            size_t target_size = 4;
            size_t result_size;
            uint32_t abort_code;

            if (ecrt_master_sdo_upload(m, arm.motor[i].buspos, 0x2F41, 0, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
            {
                fprintf(stderr, "Failed to get sdo data.\n");
                return 0;
            }

            // 2、写入0x40000 到0x2f41:0x00 配置用户自定义 pdo 的功能
            uint32_t data = 0x40000;
            uint8_t *data_send = (uint8_t *)&data;
            size_t data_size = sizeof(data);
            if (ecrt_master_sdo_download(m, arm.motor[i].buspos, 0x2f41, 0, data_send, data_size, &abort_code)) // 写SDO，0x2F41:0 16-19位配置为4 即可开启模拟二通道输入
            {
                fprintf(stderr, "Failed to Set App to analog input 2.\n");
                return 0;
            }

            // 3、重读检验是否成功写入
            if (ecrt_master_sdo_upload(m, arm.motor[i].buspos, 0x2F41, 0, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
            {
                fprintf(stderr, "Failed to get sdo data.\n");
                return 0;
            }

            // =========================================================================== //

            if (ecrt_master_sdo_upload(m, arm.motor[i].buspos, 0x6064, 0, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
            {
                fprintf(stderr, "Failed to get sdo data.\n");
                return 0;
            }
            arm.motor[i].start_pos = *((uint32_t *)result);

            if (ecrt_master_sdo_upload(m, arm.motor[i].buspos, 0x3091, 2, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
            {
                fprintf(stderr, "Failed to get sdo data.\n");
                return 0;
            }
            arm.startJointAngle[i] = 2 * PI - *((int32_t *)result) / 131072.0 * PI;  // ()/262144 * 2 * -PI
            if (i < 4)
                arm.startJointAngle[i] = 0.0;
            // if (arm.startJointAngle[i] < -PI)
            // {
            //     arm.startJointAngle[i] += 2 * PI;
            // }
            // else if (arm.startJointAngle[i] >= PI)
            // {
            //     arm.startJointAngle[i] -= 2 * PI;
            // }
            printf("sdo config OK.%d,abs pos:%f rad,rel pos:%d\n", i, arm.startJointAngle[i], arm.motor[i].start_pos);
        }

    } /* motor 循环*/

    // 初始化力传感器从站
    if (arm.motor_use[7] == 1){
        EC_position ft_pos = {motor_pos[addr].alias, motor_pos[addr].buspos};
        i = FT_sensor_init(arm, m, dm_index, ft_pos);
    }
    
    return i;
    // return 1;
}

/*
/* 初始化头电机，并将其设定为位置模式 
*  输入：记录结构体，主站序号，总线位置数组
**/
int headInit(bodypart &arm, ec_master_t *m, int dm_index, EC_position * motor_pos)
{
    arm.dm_index = dm_index;        // 记录当前身体部分所使用的domain
    arm.motornum = 0;
    arm.movefollowCnt = 0;

    int i = 0,j = 0;
    for (j = 0; j< 3; j ++)     // ！！！！！！！！！
    {
        arm.motor[j].first_time = 0;
        arm.motor[j].exp_position = 0;
        arm.motor[j].act_position = 0;
        arm.motor[j].mode = headMotorMode;
        arm.motor[j].this_send = 0;
        arm.motor[j].itp_period_times = headMotoritpTimes[j];
        arm.motor[j].plan_cnt = 0;
        arm.motor[j].plan_run_time = 0.0f;

        arm.jointGear[j] = headGear[j];
    }
    for (i = 0; i < arm.motornum; i++)     // ！！！！！！！！！
    {
        arm.motor[i].alias = motor_pos[i].alias;
        arm.motor[i].buspos = motor_pos[i].buspos;

        ec_pdo_entry_reg_t temp1 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[0].index, spe_pos[0].subindex, &arm.motor[i].offset.target_position, NULL};
        ec_pdo_entry_reg_t temp2 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[1].index, spe_pos[1].subindex, &arm.motor[i].offset.DO, NULL};
        ec_pdo_entry_reg_t temp3 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[2].index, spe_pos[2].subindex, &arm.motor[i].offset.ctrl_word, NULL};
        ec_pdo_entry_reg_t temp4 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[3].index, spe_pos[3].subindex, &arm.motor[i].offset.mode_operation, NULL};
        ec_pdo_entry_reg_t temp5 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[4].index, spe_pos[4].subindex, &arm.motor[i].offset.act_position, NULL};
        ec_pdo_entry_reg_t temp6 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[5].index, spe_pos[5].subindex, &arm.motor[i].offset.DI, NULL};
        ec_pdo_entry_reg_t temp7 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[6].index, spe_pos[6].subindex, &arm.motor[i].offset.status_word, NULL};
        ec_pdo_entry_reg_t temp8 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[7].index, spe_pos[7].subindex, &arm.motor[i].offset.ain, NULL};

        domain[dm_index].domain_reg.push_back(temp1);
        domain[dm_index].domain_reg.push_back(temp2);
        domain[dm_index].domain_reg.push_back(temp3);
        domain[dm_index].domain_reg.push_back(temp4);
        domain[dm_index].domain_reg.push_back(temp5);
        domain[dm_index].domain_reg.push_back(temp6);
        domain[dm_index].domain_reg.push_back(temp7);
        domain[dm_index].domain_reg.push_back(temp8);

        arm.motor[j].first_time = 0;
        arm.motor[j].exp_position = 0;
        arm.motor[j].act_position = 0;
        arm.motor[j].mode = armMotorMode;
        arm.motor[j].this_send = 0;
        arm.motor[j].itp_period_times = armMotoritpTimes[j];
        arm.motor[j].plan_cnt = 0;
        arm.motor[j].plan_run_time = 0.0f;

        arm.jointGear[j] = leftarmGear[j];

        arm.motor[i].sc_dig_out = ecrt_master_slave_config(m, motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD);
        if (!arm.motor[i].sc_dig_out)
        {
            fprintf(stderr, "Failed to get slave configuration, No.%d.\n", i);
            return 0;
        }
        if (ecrt_slave_config_pdos(arm.motor[i].sc_dig_out, EC_END, ss_pos))
        {
            fprintf(stderr, "Failed to configure PDOs. No.%d.\n", i);
            return 0;
        }
        // =========================================================================== //

    } /* motor 循环*/

    return 1;
}

/*
/* 初始化履带电机，并将其履带电机设定为速度模式， 关节电机设定为位置模式， 并打开analog2的输入
*  输入：记录结构体，主站序号，总线位置数组
**/
int chassisInit(bodypart &arm, trackpart & trc, ec_master_t *m, int dm_index, EC_position * motor_pos)
{
    int i = 0,j = 0;

    arm.dm_index = dm_index;        // 记录当前身体部分所使用的domain
    arm.movefollowCnt = 0;
    arm.motornum = 0;                // ！！！！！！！！！
    arm.state = IDLE;

    trc.dm_index = dm_index;
    trc.motornum = 1;               // ！！！！！！！！！
    trc.state = IDLE;

    for (j = 0; j< arm.motornum; j ++)
    {
        arm.motor[j].first_time = 0;
        arm.motor[j].exp_position = 0;
        arm.motor[j].act_position = 0;
        arm.motor[j].mode = armMotorMode;
        arm.motor[j].this_send = 0;
        arm.motor[j].itp_period_times = trackMotoritpTimes[j + 4];
        arm.motor[j].plan_cnt = 0;
        arm.motor[j].plan_run_time = 0.0f;

        arm.jointGear[j] = trackGear[j + 4];
    }

    for (j = 0; j< 4; j ++)     
    {
        trc.motor[j].first_time = 0;
        trc.motor[j].exp_velocity = 0;
        trc.motor[j].act_velocity = 0;
        trc.motor[j].mode = trackSpeedMotorMode;
        trc.motor[j].this_send = 0;
        trc.motor[j].itp_period_times = trackMotoritpTimes[j];
        trc.motor[j].plan_cnt = 0;
        trc.motor[j].plan_run_time = 0.0f;

        trc.jointGear[j] = trackGear[j];
    }

    // 初始化履带电机
    for (i = 0; i < trc.motornum; i++)     // ！！！！！！！！！ 4
    {
        trc.motor[i].alias = motor_pos[i].alias;
        trc.motor[i].buspos = motor_pos[i].buspos;

        ec_pdo_entry_reg_t temp1 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_vel[0].index, spe_vel[0].subindex, &trc.motor[i].offset.target_velocity, NULL};
        ec_pdo_entry_reg_t temp2 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_vel[1].index, spe_vel[1].subindex, &trc.motor[i].offset.ctrl_word, NULL};
        ec_pdo_entry_reg_t temp3 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_vel[2].index, spe_vel[2].subindex, &trc.motor[i].offset.mode_operation, NULL};
        ec_pdo_entry_reg_t temp4 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_vel[3].index, spe_vel[3].subindex, &trc.motor[i].offset.act_position, NULL};
        ec_pdo_entry_reg_t temp5 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_vel[4].index, spe_vel[4].subindex, &trc.motor[i].offset.DI, NULL};
        ec_pdo_entry_reg_t temp6 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_vel[5].index, spe_vel[5].subindex, &trc.motor[i].offset.act_velocity, NULL};
        ec_pdo_entry_reg_t temp7 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_vel[6].index, spe_vel[6].subindex, &trc.motor[i].offset.status_word, NULL};

        domain[dm_index].domain_reg.push_back(temp1);
        domain[dm_index].domain_reg.push_back(temp2);
        domain[dm_index].domain_reg.push_back(temp3);
        domain[dm_index].domain_reg.push_back(temp4);
        domain[dm_index].domain_reg.push_back(temp5);
        domain[dm_index].domain_reg.push_back(temp6);
        domain[dm_index].domain_reg.push_back(temp7);

        trc.motor[i].sc_dig_out = ecrt_master_slave_config(m, motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD);
        if (!trc.motor[i].sc_dig_out)
        {
            fprintf(stderr, "Failed to get slave configuration, No.%d.\n", i);
            return 0;
        }
        if (ecrt_slave_config_pdos(trc.motor[i].sc_dig_out, EC_END, ss_vels))
        {
            fprintf(stderr, "Failed to configure PDOs. No.%d.\n", i);
            return 0;
        }
    } /* motor 循环*/

    // 初始化腰部及关节电机
    for (i = 0; i < arm.motornum; i++) 
    {
        arm.motor[i].alias = motor_pos[i].alias;
        arm.motor[i].buspos = motor_pos[i].buspos;

        ec_pdo_entry_reg_t temp1 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[0].index, spe_pos[0].subindex, &arm.motor[i].offset.target_position, NULL};
        ec_pdo_entry_reg_t temp2 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[1].index, spe_pos[1].subindex, &arm.motor[i].offset.DO, NULL};
        ec_pdo_entry_reg_t temp3 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[2].index, spe_pos[2].subindex, &arm.motor[i].offset.ctrl_word, NULL};
        ec_pdo_entry_reg_t temp4 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[3].index, spe_pos[3].subindex, &arm.motor[i].offset.mode_operation, NULL};
        ec_pdo_entry_reg_t temp5 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[4].index, spe_pos[4].subindex, &arm.motor[i].offset.act_position, NULL};
        ec_pdo_entry_reg_t temp6 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[5].index, spe_pos[5].subindex, &arm.motor[i].offset.DI, NULL};
        ec_pdo_entry_reg_t temp7 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[6].index, spe_pos[6].subindex, &arm.motor[i].offset.status_word, NULL};
        ec_pdo_entry_reg_t temp8 = {motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD, spe_pos[7].index, spe_pos[7].subindex, &arm.motor[i].offset.ain, NULL};

        domain[dm_index].domain_reg.push_back(temp1);
        domain[dm_index].domain_reg.push_back(temp2);
        domain[dm_index].domain_reg.push_back(temp3);
        domain[dm_index].domain_reg.push_back(temp4);
        domain[dm_index].domain_reg.push_back(temp5);
        domain[dm_index].domain_reg.push_back(temp6);
        domain[dm_index].domain_reg.push_back(temp7);
        domain[dm_index].domain_reg.push_back(temp8);

        arm.motor[i].sc_dig_out = ecrt_master_slave_config(m, motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD);
        if (!arm.motor[i].sc_dig_out)
        {
            fprintf(stderr, "Failed to get slave configuration, No.%d.\n", i);
            return 0;
        }
        if (ecrt_slave_config_pdos(arm.motor[i].sc_dig_out, EC_END, ss_pos))
        {
            fprintf(stderr, "Failed to configure PDOs. No.%d.\n", i);
            return 0;
        }

        // ==================== 读写 SDO，配置0x2f41:0x00, 加入AD-input2到PDO中 ======================== //
        // 1、读入0x2f41:0x00 当前的值
        uint8_t result[4];
        size_t target_size = 4;
        size_t result_size;
        uint32_t abort_code;

        if (ecrt_master_sdo_upload(m, arm.motor[i].buspos, 0x2F41, 0, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
        {
            fprintf(stderr, "Failed to get sdo data.\n");
            return 0;
        }

        // 2、写入0x40000 到0x2f41:0x00 配置用户自定义 pdo 的功能
        uint32_t data = 0x40000;
        uint8_t *data_send = (uint8_t *)&data;
        size_t data_size = sizeof(data);
        if (!ecrt_master_sdo_download(m, arm.motor[i].buspos, 0x2f41, 0, data_send, data_size, &abort_code)) // 写SDO，0x2F41:0 16-19位配置为4 即可开启模拟二通道输入
        {
            printf("Set App to analog input 2 successed!\n");
        }

        // 3、重读检验是否成功写入
        if (ecrt_master_sdo_upload(m, arm.motor[i].buspos, 0x2F41, 0, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
        {
            fprintf(stderr, "Failed to get sdo data.\n");
            return 0;
        }

    } /* motor 循环*/
    
    return 1;
}
/*
 * 初始化力传感器，改变控制字使其输出力、力矩数据
 */
int FT_sensor_init(bodypart &arm, ec_master_t * m, int dm_index, EC_position pos)
{
    arm.dm_index = dm_index;
    // 将配置的总线位置记录下来
    arm.endft.pos.alias = pos.alias;
    arm.endft.pos.buspos = pos.buspos;

    // EtherCAT PDO 结构体初始化
    ec_pdo_entry_info_t spe_ft[] = {
        {0x7010, 0x01, 32}, /* Control 1 */
        {0x7010, 0x02, 32}, /* Control 2 */
        {0x6000, 0x01, 32}, /* Fx/Gage0 */
        {0x6000, 0x02, 32}, /* Fy/Gage1 */
        {0x6000, 0x03, 32}, /* Fz/Gage2 */
        {0x6000, 0x04, 32}, /* Tx/Gage3 */
        {0x6000, 0x05, 32}, /* Ty/Gage3 */
        {0x6000, 0x06, 32}, /* Tz/Gage3 */
        {0x6010, 0x00, 32}, /* SubIndex 000 */
        {0x6020, 0x00, 32}, /* SubIndex 000 */
    };

    ec_pdo_info_t sp_ft[] = {
        {0x1601, 2, spe_ft + 0}, /* DO RxPDO-Map */
        {0x1a00, 8, spe_ft + 2}, /* DI TxPDO-Map */
    };

    ec_sync_info_t ss_ft[] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, sp_ft + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 1, sp_ft + 1, EC_WD_DISABLE},
        {0xff}
    };

    ec_slave_config_t *ft_sc_dig_out;
    
    ft_sc_dig_out = ecrt_master_slave_config(m, pos.alias, pos.buspos, ATI_FTSENSOR);
    if (!ft_sc_dig_out)
    {
        fprintf(stderr, "Failed to get slave configuration, \n");
        return 0;
    }
    if (ecrt_slave_config_pdos(ft_sc_dig_out, EC_END, ss_ft))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return 0;
    }
    ec_pdo_entry_reg_t temp1 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[0].index, spe_ft[0].subindex, &arm.endft.offset.ctrl1, NULL};
    ec_pdo_entry_reg_t temp2 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[1].index, spe_ft[1].subindex, &arm.endft.offset.ctrl2, NULL};
    ec_pdo_entry_reg_t temp3 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[2].index, spe_ft[2].subindex, &arm.endft.offset.Fx, NULL};
    ec_pdo_entry_reg_t temp4 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[3].index, spe_ft[3].subindex, &arm.endft.offset.Fy, NULL};
    ec_pdo_entry_reg_t temp5 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[4].index, spe_ft[4].subindex, &arm.endft.offset.Fz, NULL};
    ec_pdo_entry_reg_t temp6 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[5].index, spe_ft[5].subindex, &arm.endft.offset.Tx, NULL};
    ec_pdo_entry_reg_t temp7 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[6].index, spe_ft[6].subindex, &arm.endft.offset.Ty, NULL};
    ec_pdo_entry_reg_t temp8 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[7].index, spe_ft[7].subindex, &arm.endft.offset.Tz, NULL};
    ec_pdo_entry_reg_t temp9 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[8].index, spe_ft[8].subindex, &arm.endft.offset.status1, NULL};
    ec_pdo_entry_reg_t temp10 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[9].index, spe_ft[9].subindex, &arm.endft.offset.status2, NULL};

    domain[dm_index].domain_reg.push_back(temp1);
    domain[dm_index].domain_reg.push_back(temp2);
    domain[dm_index].domain_reg.push_back(temp3);
    domain[dm_index].domain_reg.push_back(temp4);
    domain[dm_index].domain_reg.push_back(temp5);
    domain[dm_index].domain_reg.push_back(temp6);
    domain[dm_index].domain_reg.push_back(temp7);
    domain[dm_index].domain_reg.push_back(temp8);
    domain[dm_index].domain_reg.push_back(temp9);
    domain[dm_index].domain_reg.push_back(temp10);

    // 1、写控制字 0000 到0x7010:0x01 Sdo 功能：读出counts数据，配置滤波器为0, 校准数据为700-30
    uint32_t data1 = 0x001;
    uint32_t abort_code;
    

    uint8_t result[4];
    size_t target_size = 4;
    size_t result_size;
    uint32_t *data;

    // 2、从0x2040:0x31 读Sdo 得到Counts per Force
    if (ecrt_master_sdo_upload(m, pos.buspos, 0x2040, 0x31, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
    {
        fprintf(stderr, "Failed to get sdo data.\n");
        return 0;
    }
    data = (uint32_t *)result;
    arm.endft.countsPerForce = double(*data);      // 将读到的数据写入手臂结构体
    // printf("%f,%d\n",(double(*data)), result_size);

    // 3、从0x2040:0x32 读Sdo 得到Counts per Torque
    if (ecrt_master_sdo_upload(m, pos.buspos, 0x2040, 0x32, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
    {
        fprintf(stderr, "Failed to get sdo data.\n");
        return 0;
    }
    data = (uint32_t *)result;
    arm.endft.countsPerTorque = double(*data);      // 将读到的数据写入手臂结构体
    // printf("%f,%d\n",(double(*data)), result_size);

    if (ecrt_master_sdo_download(m, pos.buspos, 0x7010, 0x01, (uint8_t *)&data1, sizeof(data1), &abort_code)) // 写SDO，0x2F41:0 16-19位配置为4 即可开启模拟二通道输入
    {
        fprintf(stderr, "Failed to set Control Word\n");
        return 0;
    }

    memset(arm.endft.offsetft, 0, sizeof(arm.endft.offsetft));      // 将偏置设为零
    
    return 1;

    // 4、从0x6000:0x03 读Sdo 试读力传感器数据Fz
    // if (ecrt_master_sdo_upload(m, pos.buspos, 0x6000, 0x03, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
    // {
    //     fprintf(stderr, "Failed to get sdo data.\n");
    //     return -1;
    // }
    // data1 = result[3] << 24 | result[2] << 16 | result[1] << 8 | result[0];
    // printf("%f\n",(float)data1/1000000.0);

}

/*
 * 处理domain，当前多个domain，每个domain只需要pushback即可
 */
int ArrayDomainRegs(EC_domain &dm)
{
    int i = 0;
    // for (i = 0; i < dm.domain_reg.size(); i++)
    // {
    //     memcpy(&dm.domain_regs[i], &dm.domain_reg.at(i), sizeof(dm.domain_reg.at(i)));
    // }
    // ec_pdo_entry_reg_t temp = {};
    // memcpy(&dm.domain_regs[i], &temp, sizeof(temp));
    ec_pdo_entry_reg_t *buffer = new ec_pdo_entry_reg_t[dm.domain_reg.size() + 1];  
    if (!dm.domain_reg.empty())  
    {  
        memcpy(buffer, &dm.domain_reg[0], dm.domain_reg.size()*sizeof(ec_pdo_entry_reg_t));  
    }  

    ec_pdo_entry_reg_t temp = {};
    memcpy(&buffer[dm.domain_reg.size()], &temp, sizeof(temp));

    i = ecrt_domain_reg_pdo_entry_list(dm.domain, buffer);
    free(buffer);
    return i;
}

/*
 * 三次多项式插值 位置电机
 */
int interpolation(Motor &m)
{
    double p1 = 0;
    double p2 = 0;
    double v1 = 0;
    double v2 = 0;
    double tf = double(m.itp_period_times) * double(ctl_period) / 1e9;

    while (m.plan.size() <= itp_window + 1)
    {
        m.plan.push_back(m.exp_position);
    }

    if (m.plan.size() > itp_window + 1) //保持itp_window+1长度的队列
    {
        m.plan.erase(m.plan.begin());

        for (int j = 0; j < itp_window; j++) //遍历列表取数
        {
            p1 += m.plan[j] * alpha[j] / alpha_sum;
            p2 += m.plan[j + 1] * alpha[j] / alpha_sum;
        }
        v1 = (m.plan[itp_window - 1] - m.plan[0]) / ((itp_window - 1) * tf);
        v2 = (m.plan[itp_window] - m.plan[1]) / ((itp_window - 1) * tf);

        m.plan_param[0] = p1;
        m.plan_param[1] = v1;
        m.plan_param[2] = 3.0f / (tf * tf) * (p2 - p1) - 2.0f / tf * v1 - 1.0f / tf * v2;
        m.plan_param[3] = -2.0f / (tf * tf * tf) * (p2 - p1) + 1.0f / (tf * tf) * (v2 + v1);
    }
}

/*
 * 三次多项式插值 速度电机
 */
int interpolation_v(velMotor &m)
{
    double p1 = 0;
    double p2 = 0;
    double v1 = 0;
    double v2 = 0;
    double tf = double(m.itp_period_times) * double(ctl_period) / 1e9;

    while (m.plan.size() <= itp_window + 1)
    {
        m.plan.push_back(m.exp_velocity);
    }

    if (m.plan.size() > itp_window + 1) //保持itp_window+1长度的队列
    {
        m.plan.erase(m.plan.begin());

        for (int j = 0; j < itp_window; j++) //遍历列表取数
        {
            p1 += m.plan[j] * alpha[j] / alpha_sum;
            p2 += m.plan[j + 1] * alpha[j] / alpha_sum;
        }
        v1 = (m.plan[itp_window - 1] - m.plan[0]) / ((itp_window - 1) * tf);
        v2 = (m.plan[itp_window] - m.plan[1]) / ((itp_window - 1) * tf);

        m.plan_param[0] = p1;
        m.plan_param[1] = v1;
        m.plan_param[2] = 3.0f / (tf * tf) * (p2 - p1) - 2.0f / tf * v1 - 1.0f / tf * v2;
        m.plan_param[3] = -2.0f / (tf * tf * tf) * (p2 - p1) + 1.0f / (tf * tf) * (v2 + v1);
    }
}

/*
 * EtherCat写帧-改变电机状态-状态机
 */
uint8_t changeOneMotorState(bodypart &arm, int8_t id, uint8_t state)
{
    Motor *m = &arm.motor[id];
    int dm_index = arm.dm_index;
    /* read inputs */
    m->status = EC_READ_U16(domain[dm_index].domain_pd + m->offset.status_word);
    // printf("%x\n",m->status);

    //DS402 CANOpen over EtherCAT status machine
    if ((m->status & 0x004f) == 0x0008) // Fault
    {
        EC_WRITE_U16(domain[dm_index].domain_pd + m->offset.ctrl_word, FaultReset);
        return 0;
    }

    if ((m->status & 0x004f) == 0x0040) // Switch on disable
    {
        if (state == SWITCH_ON_DISABLED)
        {
            return 1;
        }
        else
        {
            EC_WRITE_U16(domain[dm_index].domain_pd + m->offset.ctrl_word, SM_trans2);
        }
    }
    else if ((m->status & 0x006f) == 0x0021) //Ready to switch on
    {
        if (state == READY_TO_SWITCH_ON)
        {
            return 1;
        }
        else
        {
            EC_WRITE_U16(domain[dm_index].domain_pd + m->offset.ctrl_word, SM_trans3);
        }
    }
    else if ((m->status & 0x006f) == 0x0023) //switch on
    {
        if (state == SWITCHED_ON)
        {
            return 1;
        }
        else
        {
            EC_WRITE_U16(domain[dm_index].domain_pd + m->offset.ctrl_word, SM_trans4);
        }
    }
    else if ((m->status & 0x006f) == 0x0027) //operation enabled
    {
        if (state == OPERATION_ENABLE)
        {
            return 1;
        }
        else if (state == SWITCHED_ON)
        {
            EC_WRITE_U16(domain[dm_index].domain_pd + m->offset.ctrl_word, SM_trans5);
        }
        else if (state == QUICK_STOP_ACTIVE || state == SWITCH_ON_DISABLED)
        {
            EC_WRITE_U16(domain[dm_index].domain_pd + m->offset.ctrl_word, QuickStop);
        }
    }
    return 0;
}

/*
 * EtherCat写帧-改变多个电机状态
 */
uint8_t changeBodyMotorState(bodypart &arm, int8_t id, uint8_t state)
{
    int i = 0;
    uint8_t ret = 1;
    uint8_t ret_t;
    if (id == -1)
    {
        for (i = 0; i < arm.motornum; i++)
        {
            ret_t = changeOneMotorState(arm, i, state);
            ret &= ret_t;
        }
    }
    else
    {
        ret = changeOneMotorState(arm, id, state);
    }
    return ret;
}

uint8_t changeOneTrackMotorState(trackpart &trc, int8_t id, uint8_t state)
{
    velMotor *m = &trc.motor[id];
    int dm_index = trc.dm_index;
    /* read inputs */
    m->status = EC_READ_U16(domain[dm_index].domain_pd + m->offset.status_word);
    // printf("%x\n",m->status);

    //DS402 CANOpen over EtherCAT status machine
    if ((m->status & 0x004f) == 0x0008) // Fault
    {
        EC_WRITE_U16(domain[dm_index].domain_pd + m->offset.ctrl_word, FaultReset);
        return 0;
    }

    if ((m->status & 0x004f) == 0x0040) // Switch on disable
    {
        if (state == SWITCH_ON_DISABLED)
        {
            return 1;
        }
        else
        {
            EC_WRITE_U16(domain[dm_index].domain_pd + m->offset.ctrl_word, SM_trans2);
        }
    }
    else if ((m->status & 0x006f) == 0x0021) //Ready to switch on
    {
        if (state == READY_TO_SWITCH_ON)
        {
            return 1;
        }
        else
        {
            EC_WRITE_U16(domain[dm_index].domain_pd + m->offset.ctrl_word, SM_trans3);
        }
    }
    else if ((m->status & 0x006f) == 0x0023) //switch on
    {
        if (state == SWITCHED_ON)
        {
            return 1;
        }
        else
        {
            EC_WRITE_U16(domain[dm_index].domain_pd + m->offset.ctrl_word, SM_trans4);
        }
    }
    else if ((m->status & 0x006f) == 0x0027) //operation enabled
    {
        if (state == OPERATION_ENABLE)
        {
            return 1;
        }
        else if (state == SWITCHED_ON)
        {
            EC_WRITE_U16(domain[dm_index].domain_pd + m->offset.ctrl_word, SM_trans5);
        }
        else if (state == QUICK_STOP_ACTIVE || state == SWITCH_ON_DISABLED)
        {
            EC_WRITE_U16(domain[dm_index].domain_pd + m->offset.ctrl_word, QuickStop);
        }
    }
    return 0;
}

uint8_t changeTrackMotorState(trackpart &trc, int8_t id, uint8_t state)
{
    int i = 0;
    uint8_t ret = 1;
    uint8_t ret_t;
    if (id == -1)
    {
        for (i = 0; i < trc.motornum; i++)
        {
            ret_t = changeOneTrackMotorState(trc, i, state);
            ret &= ret_t;
        }
    }
    else
    {
        ret = changeOneTrackMotorState(trc, id, state);
    }
    return ret;
}

/*
 * 停止手臂电机运动，输入左臂或右臂
 */
void stopArmMotor(bodypart & arm)
{
    int i;
    int motornum = arm.motornum;
    for (i = 0; i < motornum; i++)
    {
        arm.motor[i].exp_position = arm.motor[i].act_position;
        arm.motor[i].plan_cnt = 0;
        arm.motor[i].plan.clear();
    }
    arm.state = IDLE;
}

/*
 * 读手臂电机反馈数据，输入左臂或右臂
 */
void readArmData(bodypart & arm)
{
    // printf("%f,", arm.motor[0].ref_position);

    int i;
    int motornum = arm.motornum;
    for (i = 0; i < motornum; i++)
    {
        if (arm.motor_use[i] == 1)
        {
            arm.motor[i].act_position = EC_READ_S32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.act_position) - arm.motor[i].start_pos + int((arm.startJointAngle[i] - arm.offsetAngle[i]) * arm.jointGear[i]);
        }
        else
        {
            arm.motor[i].act_position = 0;
        }
        
        // printf("%d\n", arm.motor[i].act_position);
        arm.motor[i].ain = EC_READ_U32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.ain);
        if (arm.motor[i].first_time == 0) // 初次进入，记录开机时刻位置作为期望位置
        {
            printf("first:%d, act_position:%.2f degree\n", i, arm.motor[i].act_position / arm.jointGear[i] * RAD2DEG);
            arm.motor[i].exp_position = arm.motor[i].act_position;
            arm.motor[i].ref_position = arm.motor[i].act_position;
            arm.motor[i].first_time = 1;
        }
        arm.jointPos[i] = (double)(arm.motor[i].act_position) / arm.jointGear[i] ;

        // printf("%f\n", arm.jointPos[i]);

    }
    // printf("%f\n", arm.motor[0].ref_position);
    // printf("%f, %f, %f, %f, %f, %f, %f\n", arm.motor[0].act_position, arm.motor[1].act_position, arm.motor[2].act_position, arm.motor[3].act_position, arm.motor[4].act_position, arm.motor[5].act_position,  arm.motor[6].act_position);

    readForceData(arm);
}

/*
 * 读力传感器数据，输入左臂或右臂
 */
void readForceData(bodypart &arm)
{
    int dm_index = arm.dm_index;
    arm.endft.ft[0] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Fx)/arm.endft.countsPerForce - arm.endft.offsetft[0];
    arm.endft.ft[1] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Fy)/arm.endft.countsPerForce - arm.endft.offsetft[1];
    arm.endft.ft[2] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Fz)/arm.endft.countsPerForce - arm.endft.offsetft[2];
    arm.endft.ft[3] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Tx)/arm.endft.countsPerTorque- arm.endft.offsetft[3];
    arm.endft.ft[4] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Ty)/arm.endft.countsPerTorque- arm.endft.offsetft[4];
    arm.endft.ft[5] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Tz)/arm.endft.countsPerTorque- arm.endft.offsetft[5];

    arm.endft.status = EC_READ_U32(domain[dm_index].domain_pd + arm.endft.offset.status1);
    arm.endft.sampCount = EC_READ_U32(domain[dm_index].domain_pd + arm.endft.offset.status2);

    int i = 0 ;
    double sum = 0.0;
    for (i = 0; i < 6; i++)
    {
        sum += fabs(arm.endft.ft[i]);
    }
    if (sum != 0.0)
    {
        arm.endft.dataReady = 1;
    }
    
    // printf("%f,%f,%f,%f,%f,%f, %d\n",arm.endft.ft[0], arm.endft.ft[1], arm.endft.ft[2], arm.endft.ft[3], arm.endft.ft[4], arm.endft.ft[5], arm.endft.status);
}

/*
 * 清零力传感器数据，输入力传感器结构
 */
void clearForceSensor(ft_sensor &endft)
{
    // uint32_t data1 = 0x001;
    // EC_WRITE_S32(domain[arm.dm_index].domain_pd + arm.endft.offset.ctrl1, data1);        // 直接修改力传感器偏置后数据不正常, 采用下面的记录偏置方法
    int i;
    for ( i = 0; i<6; i++)
    {
        endft.offsetft[i] += endft.ft[i];
    }
}

/*
 * 读手臂电机反馈数据，输入左臂或右臂
 */
void readChassisData(bodypart & leg, trackpart & trc)
{
    int i;
    for (i = 0; i < 0; i++)
    {
        // if (i == 0){
            // leg.motor[i].act_position = EC_READ_S32(domain[leg.dm_index].domain_pd + leg.motor[i].offset.act_position);
        // }
        // else{
            leg.motor[i].act_position = leg.motor[i].this_send;
        // }

        leg.motor[i].ain = EC_READ_U32(domain[leg.dm_index].domain_pd + leg.motor[i].offset.ain);
        if (leg.motor[i].first_time == 0) // 初次进入，记录开机时刻位置作为期望位置
        {
            printf("first:%d, act_position:%d\n", i, leg.motor[i].act_position);
            leg.state = IDLE;
            leg.motor[i].exp_position = leg.motor[i].act_position;
            leg.motor[i].first_time = 1;
        }
        leg.jointPos[i] = (double)leg.motor[i].act_position / leg.jointGear[i];
    }

    for (i = 0; i < trc.motornum; i++)
    {
        // if (i == 0){
        //     trc.motor[i].act_velocity = EC_READ_S32(domain[trc.dm_index].domain_pd + trc.motor[i].offset.act_velocity);
        // }
        // else{
            trc.motor[i].act_velocity = trc.motor[i].this_send;
        // }

        if (trc.motor[i].first_time == 0) // 初次进入，记录开机时刻位置作为期望位置
        {
            printf("first:%d, act_velcity:%d\n", i, trc.motor[i].act_velocity);
            trc.state = IDLE;
            trc.motor[i].exp_velocity = trc.motor[i].act_velocity;
            trc.motor[i].first_time = 1;
        }
        trc.jointVel[i] = (double)trc.motor[i].act_velocity / trc.jointGear[i];
    }
}

/*
 * 控制手臂电机，输入左臂或右臂
 */
void ctrlArmMotor(bodypart &arm)
{
    int i, j;
    int motornum = arm.motornum;

    // moveL 临时变量
    double location[3];
    double t_line;
    double beta_line;
    double equat_line[4];
    double angle_planned[8];
    int angle_planned_size[2];
    double angle_delta[7];
    double R[9];
    double rot_[9];
    double pose_line[16];
    double maxAngle = 0;
    uint8_t servoCmdAll = 0;
    uint8_t servoStateAll = 0;
    int ret;
    
    switch (arm.state) // 根据不同功能得到motor.ref_position的值，进行不同种类的控制
    {
        case DISABLE:       // 整臂去使能状态
            for (i = 0; i< motornum; i++)
            {
                if (arm.motor[i].servo_state == 1)
                {
                    ret = changeOneMotorState(arm, i, SWITCHED_ON);
                    if (ret)
                        arm.motor[i].servo_state = 0;
                }
            }
        break;

        case IDLE:      // 空闲状态，可以进行任务
            arm.test_time = 0.0;
            servoCmdAll = 0;
            servoStateAll = 0;
            for (i = 0; i< motornum; i++)
            {
                if (arm.motor_use[i] == 1)
                {
                    if (arm.motor[i].servo_cmd == 1 && arm.motor[i].servo_state == 0)
                    {
                        arm.motor[i].ref_position = arm.motor[i].act_position;      // 清除遗留目标位置
                        ret = changeOneMotorState(arm, i, OPERATION_ENABLE);
                        if (ret)
                            arm.motor[i].servo_state = 1;
                    }
                    else if (arm.motor[i].servo_cmd == 0 && arm.motor[i].servo_state == 1)
                    {
                        ret = changeOneMotorState(arm, i, SWITCHED_ON);
                        if (ret)
                            arm.motor[i].servo_state = 0;
                    }
                }
                
                servoCmdAll |= arm.motor[i].servo_cmd;      // 检测全零
                servoStateAll |= arm.motor[i].servo_state;
            }
            if (servoCmdAll == 0 && servoStateAll == 0)
            {
                printf("out,\n");
                arm.state = DISABLE;
            }
        break;

        case ON_MOVEL:
            if (arm.s_planTimes < 0)        // 规划时间完毕，回到IDLE状态
            {
                printf("moveL finished\n");
                arm.state = IDLE;
            }
            
            if (arm.plan_cnt == 0)      //整臂s曲线规划周期到达，进行插值规划
            {
                t_line = S_position(arm.s_line.time, arm.s_line.para);
                location[0] = arm.locationInit[0] + arm.locationDelta[0] * t_line;
                location[1] = arm.locationInit[1] + arm.locationDelta[1] * t_line;
                location[2] = arm.locationInit[2] + arm.locationDelta[2] * t_line;

                beta_line = S_position(arm.s_beta.time, arm.s_beta.para);
                equat_line[0] = S_position(arm.s_equat.time, arm.s_equat.para);
                equat_line[1] = arm.rEquivalent[1];
                equat_line[2] = arm.rEquivalent[2];
                equat_line[3] = arm.rEquivalent[3];

                // 将当前位置与初始姿态组成位姿矩阵
                quat2rot(equat_line, rot_);
                matrixMultiply(rot_, 3, 3, arm.rotInit, 3, 3, R);
                TfromRotPos(R, location, pose_line);

                InverseKinematics(arm.jointPos, pose_line, beta_line, 0, beta_line, angle_planned, angle_planned_size);

                if (angle_planned_size[1] == 8)
                {
                    for (i = 0; i < motornum; i++)
                    {
                        arm.motor[i].ref_position = angle_planned[i] * arm.jointGear[i];
                        // if (j ==0) printf("%f,%f,%f\n",angle_planned[0], arm.jointGear[0], arm.jointPos[0]);
                        angle_delta[i] = fabs(angle_planned[i] - arm.jointPos[i]);
                    }
                }
                else{
                    printf("Inverse Kinematics Failed\n");
                }
                
                arm.s_line.time += arm.s_line.deltaTime;
                arm.s_beta.time += arm.s_beta.deltaTime;
                arm.s_equat.time += arm.s_equat.deltaTime;

                arm.s_planTimes --;
                printf("%d\n",arm.s_planTimes);
            }       // 规划得到了每个电机的ref_position
            // 规划周期计数器，itp_period_times次触发
            arm.plan_cnt ++ ;
            if (arm.plan_cnt == arm.itp_period_times)
            {
                arm.plan_cnt = 0;
            }

            // 进行运动保护，1ms位置差大于某一rad报警，值需要调整
            maxAngle = max(angle_delta, 7);
            if (maxAngle > 0.05)
            {
                printf("may fail when excuting,%f\n", maxAngle);
                arm.state = IDLE;       // 退出moveL的执行
            }

        break;
        
        case ON_MOVEJ:
            if (arm.s_planTimes < 0)        // 规划时间完毕，回到IDLE状态
            {
                printf("moveJ finished\n");
                arm.state = IDLE;
            }
        
            if (arm.plan_cnt == 0)  //规划周期到达，进行插值规划， 更新ref_position
            {
                for ( i = 0; i< motornum; i++)
                {
                    arm.motor[i].ref_position = S_position(arm.motor[i].sp.time, arm.motor[i].sp.para) * arm.jointGear[i];
                    arm.motor[i].sp.time += arm.motor[i].sp.deltaTime;
                }
                arm.s_planTimes --;
                printf("%d\n",arm.s_planTimes);     // 倒计时
            }

            // 规划周期计数器，itp_period_times次触发
            arm.plan_cnt ++ ;
            if (arm.plan_cnt == arm.itp_period_times)
            {
                arm.plan_cnt = 0;
            }

            break;

        case ON_MOVETEST:
            // test计时器累加
            arm.test_time += 0.001;
        break;

        case ON_MOVE_FOLLOW:
            // 超时检测，在接收到新的moveFollow指令清除看门狗
            check_follow(arm, 0.5);
            break;

        default:
            break;
    }

    // 经过力控制的滤波器，对ref_position叠加一个力偏置得到期望电机position
    int dir_enalbe[6] = {1, 0, 0, 0, 0, 0};     // 力控制笛卡尔空间使能
    uint8_t forceCtrlType = 2;      // 0 逆运动学直接模式， 1 逆运动学差分模式（卡顿）， 2 雅克比模式（不稳定）

    // 力控状态机，由于没有状态切换之间的等待， 所以只使用命令字表示状态
    if (arm.fctrl.Switch == 1)      // 力控打开
    {
        forceUpdate(arm, forceCtrlType, 0.001, dir_enalbe);  // deltaT in 秒
    }
    else if (arm.fctrl.Switch == -1)        // 力控关闭准备
    {
        if (forceUpdate(arm, forceCtrlType, 0.001, dir_enalbe) == 0)        // 力控偏置输出为零
        {
            arm.fctrl.Switch = 0;       // 运动结束，正式关闭
            printf("closed forceCtrl\n");
        }
    }
    else if (arm.fctrl.Switch == -2)    // 力控打开准备, 如果力传感器数据已经传回, 则直接通过，否则等待启动
    {
        if (arm.endft.dataReady)
            arm.fctrl.Switch = 1;
        else
            printf("no force data\n");
    }
    else if (arm.fctrl.Switch == 0)     // 力控关闭，参考位置直接进行插值
    {
        for (i = 0 ; i < motornum; i++)
        {
            arm.motor[i].exp_position = arm.motor[i].ref_position  + arm.test_A / 2.0 * leftarmGear[i] * (1 - cos(arm.test_time * PI / arm.test_T));
        }
    }
    
    // printf("%f\n",arm.motor[0].exp_position);

    // printf("%.2f,%.2f,%.2f\n", double(arm.motor[1].act_position)/arm.jointGear[1]*RAD2DEG,  arm.motor[1].exp_position/arm.jointGear[1]*RAD2DEG, arm.motor[i].exp_position + arm.motor[1].start_pos - (arm.startJointAngle[1] - arm.offsetAngle[1]) * arm.jointGear[1]);

    // 电机遍历取值，精插补
    for (i = 0; i < motornum; i++)
    {
        /********************* 电机轨迹精插值规划 **********************/
        if (arm.motor[i].plan_cnt == 0)
        {
            interpolation(arm.motor[i]);
        }
        /********************* 电机轨迹精插值运动 **********************/
        arm.motor[i].plan_run_time = double(arm.motor[i].plan_cnt) * double(ctl_period) / 1e9;
        arm.motor[i].this_send = arm.motor[i].plan_param[0] + arm.motor[i].plan_param[1] * arm.motor[i].plan_run_time + arm.motor[i].plan_param[2] * arm.motor[i].plan_run_time * arm.motor[i].plan_run_time + arm.motor[i].plan_param[3] * arm.motor[i].plan_run_time * arm.motor[i].plan_run_time * arm.motor[i].plan_run_time;

        arm.motor[i].plan_cnt++;
        if (arm.motor[i].plan_cnt == arm.motor[i].itp_period_times)
        {
            arm.motor[i].plan_cnt = 0;
        }

        // if ( i == 6)
        // {
        //     printf("%d,%f\n", arm.motor[i].act_position, arm.motor[i].this_send);
        // }
        /********************** 填写指令，等待发送 **********************/
        // if (i == -1){
        //     // printf("%d\n",int(arm.motor[i].this_send));
        //     if (arm.state != ON_MOVE_FOLLOW)
        if (arm.motor_use[i] == 1){
            EC_WRITE_S32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.target_position, int(arm.motor[i].this_send) + arm.motor[i].start_pos - int((arm.startJointAngle[i] - arm.offsetAngle[i]) * arm.jointGear[i]));
        }
        // }

    }
}

/*
 * 控制头电机，输入头
 */
void ctrlHeadMotor(bodypart &head)
{
    int i, j;
    int motornum = head.motornum;
    int ret;
    int servoall = 0;

    switch (head.state)
    {

        case DISABLE:       // 整臂去使能状态
            changeBodyMotorState(head, -1, SWITCHED_ON);
            break;

        case IDLE:      // 空闲状态，可以进行任务

            for (i = 0; i< motornum; i++)
            {
                if (head.motor[i].servo_cmd == 1)
                {
                    head.motor[i].ref_position = head.motor[i].act_position;      // 清除遗留目标位置
                    ret = changeBodyMotorState(head, i, OPERATION_ENABLE);
                    if (ret)
                        head.motor[i].servo_state = 1;
                    else 
                        head.motor[i].servo_state = 0;
                }
                else if (head.motor[i].servo_cmd == 0)
                {
                    ret = changeBodyMotorState(head, i, SWITCHED_ON);
                    if (ret)
                        head.motor[i].servo_state = 0;
                }

                servoall |= head.motor[i].servo_cmd;
                if (servoall == 0)
                {
                    head.state = DISABLE;
                }
            }
            break;

        case ON_MOVEL:
            break;
        
        case ON_MOVEJ:
            if (head.s_planTimes < 0)        // 规划时间完毕
            {
                head.state = IDLE;
            }
        
            if (head.plan_cnt == 0) //规划周期到达，进行插值规划， 更新exp_position
            {
                for ( i = 0; i< motornum; i++)
                {
                    /********************* 电机轨迹S曲线插值运动 **********************/
                    head.motor[i].exp_position = S_position(head.motor[i].sp.time, head.motor[i].sp.para) * head.jointGear[i];
                    head.motor[i].sp.time += head.motor[i].sp.deltaTime;
                }
                head.s_planTimes --;
                printf("%d\n",head.s_planTimes);
            }

            head.plan_cnt ++ ;
            if (head.plan_cnt == head.itp_period_times)
            {
                head.plan_cnt = 0;
            }
            
            break;

        case ON_MOVE_FOLLOW:
            check_follow(head, 0.5);
            break;
        default:
            break;
    }

    for (i = 0; i < motornum; i++)
    {
        if (head.motor[i].plan_cnt == 0) //规划周期到达，进行插值规划
        {
            /********************* 电机轨迹精插值运动 **********************/
            interpolation(head.motor[i]);
        }
        head.motor[i].plan_run_time = double(head.motor[i].plan_cnt) * double(ctl_period) / 1e9;
        head.motor[i].this_send = head.motor[i].plan_param[0] + head.motor[i].plan_param[1] * head.motor[i].plan_run_time + head.motor[i].plan_param[2] * head.motor[i].plan_run_time * head.motor[i].plan_run_time + head.motor[i].plan_param[3] * head.motor[i].plan_run_time * head.motor[i].plan_run_time * head.motor[i].plan_run_time;

        head.motor[i].plan_cnt++;
        if (head.motor[i].plan_cnt == head.motor[i].itp_period_times)
        {
            head.motor[i].plan_cnt = 0;
        }

        /********************** 填写指令，等待发送 **********************/  // ！！！！！！！！！
        // EC_WRITE_S32(domain[head.dm_index].domain_pd + head.motor[i].offset.target_position, int(head.motor[i].this_send));
    }

}

/*
 * 控制下肢电机，输入下肢
 */
void ctrlLegMotor(bodypart &leg)
{
    int i, j;
    int motornum = leg.motornum;
    int ret;
    int servoall = 0;
   
    switch (leg.state)
    {
    case DISABLE:       // 整臂去使能状态
        changeBodyMotorState(leg, -1, SWITCHED_ON);
        break;

    case IDLE:      // 空闲状态，可以进行任务

        for (i = 0; i< motornum; i++)
        {
            if (leg.motor[i].servo_cmd == 1)
            {
                leg.motor[i].ref_position = leg.motor[i].act_position;      // 清除遗留目标位置
                ret = changeBodyMotorState(leg, i, OPERATION_ENABLE);
                if (ret)
                    leg.motor[i].servo_state = 1;
                else 
                    leg.motor[i].servo_state = 0;
            }
            else if (leg.motor[i].servo_cmd == 0)
            {
                ret = changeBodyMotorState(leg, i, SWITCHED_ON);
                if (ret)
                    leg.motor[i].servo_state = 0;
            }

            servoall |= leg.motor[i].servo_cmd;
            if (servoall == 0)
            {
                leg.state = DISABLE;
            }
        }
        break;

    case ON_MOVEL:

        break;
    
    case ON_MOVEJ:
        if (leg.s_planTimes < 0)        // 规划时间完毕
        {
            leg.state = IDLE;
        }
    
        if (leg.plan_cnt == 0) //规划周期到达，进行插值规划， 更新exp_position
        {
            for ( i = 0; i< motornum; i++)
            {
                /********************* 电机轨迹S曲线插值运动 **********************/
                leg.motor[i].exp_position = S_position(leg.motor[i].sp.time, leg.motor[i].sp.para) * leg.jointGear[i];
                leg.motor[i].sp.time += leg.motor[i].sp.deltaTime;
            }
            leg.s_planTimes --;
            printf("%d\n",leg.s_planTimes);
        }

        leg.plan_cnt ++ ;
        if (leg.plan_cnt == leg.itp_period_times)
        {
            leg.plan_cnt = 0;
        }
        
        break;

    default:
        break;
    }

    for (i = 0; i < motornum; i++)
    {
        if (leg.motor[i].plan_cnt == 0) //规划周期到达，进行插值规划
        {
            /********************* 电机轨迹精插值运动 **********************/
            interpolation(leg.motor[i]);
        }
        leg.motor[i].plan_run_time = double(leg.motor[i].plan_cnt) * double(ctl_period) / 1e9;
        leg.motor[i].this_send = leg.motor[i].plan_param[0] + leg.motor[i].plan_param[1] * leg.motor[i].plan_run_time + leg.motor[i].plan_param[2] * leg.motor[i].plan_run_time * leg.motor[i].plan_run_time + leg.motor[i].plan_param[3] * leg.motor[i].plan_run_time * leg.motor[i].plan_run_time * leg.motor[i].plan_run_time;

        leg.motor[i].plan_cnt++;
        if (leg.motor[i].plan_cnt == leg.motor[i].itp_period_times)
        {
            leg.motor[i].plan_cnt = 0;
        }

        /********************** 填写指令，等待发送 **********************/ // ！！！！！！！！！
        // EC_WRITE_S32(domain[leg.dm_index].domain_pd + leg.motor[i].offset.target_position, int(leg.motor[i].this_send));
    }

}

/*
 * 控制履带电机，输入履带
 */
void ctrlTrackMotor(trackpart &trc)
{
    int i, j;

    switch (trc.state)
    {
    case ON_MOVEL:

        break;
    
    case ON_CARMOVE:
        printf_d(trc.jointGear,4);
        trc.motor[0].exp_velocity = (trc.chassisVel_cmd[0] - trc.chassisVel_cmd[1] * R1) * trc.jointGear[0];        // trc.chassisVel_cmd 单位 mm，rad/s, 乘以系数 临时 cnt/s
        trc.motor[1].exp_velocity = (trc.chassisVel_cmd[0] - trc.chassisVel_cmd[1] * R2) * trc.jointGear[1];
        trc.motor[2].exp_velocity = (trc.chassisVel_cmd[0] + trc.chassisVel_cmd[1] * R2) * trc.jointGear[2];
        trc.motor[3].exp_velocity = (trc.chassisVel_cmd[0] + trc.chassisVel_cmd[1] * R1) * trc.jointGear[3];
        trc.watchdog ++;
        // printf("%f,%f\n",trc.motor[0].exp_velocity, trc.motor[1].exp_velocity);
        if (trc.watchdog > 5 * 1000)
        {
            for (i = 0; i< 4; i++)
            {
                trc.motor[i].exp_velocity = 0.0;
            }
            trc.state = IDLE;
            trc.watchdog = 0;
            printf("out\n");
        }
        break;

    default:
        break;
    }

    for (i = 0; i < trc.motornum; i++)
    {
        if (trc.motor[i].plan_cnt == 0) //规划周期到达，进行插值规划
        {
            /********************* 电机轨迹精插值运动 **********************/
            interpolation_v(trc.motor[i]);
        }
        trc.motor[i].plan_run_time = double(trc.motor[i].plan_cnt) * double(ctl_period) / 1e9;
        trc.motor[i].this_send = trc.motor[i].plan_param[0] + trc.motor[i].plan_param[1] * trc.motor[i].plan_run_time + trc.motor[i].plan_param[2] * trc.motor[i].plan_run_time * trc.motor[i].plan_run_time + trc.motor[i].plan_param[3] * trc.motor[i].plan_run_time * trc.motor[i].plan_run_time * trc.motor[i].plan_run_time;

        trc.motor[i].plan_cnt++;
        if (trc.motor[i].plan_cnt == trc.motor[i].itp_period_times)
        {
            trc.motor[i].plan_cnt = 0;
        }
        /********************** 填写指令，等待发送 **********************/ // ！！！！！！！！！
        // EC_WRITE_S32(domain[trc.dm_index].domain_pd + trc.motor[i].offset.target_velocity, int(trc.motor[i].this_send));
    }
   

}

// 检查follow超时，timeout单位为秒
void check_follow(bodypart & arm, double timeout)
{
    if (arm.state == ON_MOVE_FOLLOW)
    {
        arm.movefollowCnt ++;
        if (arm.movefollowCnt >= (int)(timeout / ((double)ctl_period /1e9 )) )
        {
            arm.movefollowCnt = 0;
            arm.state = IDLE;
        }
    }
}

/**
 * 实时任务函数
 */
void realtime_proc(void *arg)
{
    // 系统时间
    RTIME now, previous, period;
    rt_task_set_periodic(NULL, TM_NOW, RTIME(ctl_period)); // unit :ns
    uint8_t run_state = CONFIG_ELMO;

    int i, ret;
    uint8_t ready = 0;
    command cmd;
    int left_right;
    double speedRate;

    int enable_id;
    // moveJ
    double jointFinal[7] = {0.0};
    double jointFinalBeta[8] = {0.0};
    double poseFinal[6] = {0.0};
    double Tfinal[16] = {0.0};
    int angleFianl_beta_size[2] = {0};

    // force
    double force_param;

    // test 
    double test_A = 0;
    double test_T = 0;

    while (run)
    {
        rt_task_wait_period(NULL);      // 等待定时周期
        previous = rt_timer_read();     // 获取运行周期开始时间

        // 接收 EtherCAT 帧
        ecrt_master_receive(master0);
        ecrt_master_receive(master1);

        ecrt_domain_process(domain[0].domain);
        ecrt_domain_process(domain[1].domain);

        // readForceData(leftarm);

        switch (run_state)
        {

        case CONFIG_ELMO:
            ready = 1;
            for (i = 0; i < 7; i++)
            {
                EC_WRITE_U8(domain[rightarm.dm_index].domain_pd + rightarm.motor[i].offset.mode_operation, rightarm.motor[i].mode);
                EC_WRITE_U8(domain[leftarm.dm_index].domain_pd + leftarm.motor[i].offset.mode_operation, leftarm.motor[i].mode);
                if (leftarm.motor_use[i] == 1){
                    ready &= changeOneMotorState(leftarm, i, SWITCHED_ON);
                }
                if (rightarm.motor_use[i] == 1){
                    ready &= changeOneMotorState(rightarm, i, SWITCHED_ON);
                }
            }

            // for ( i = 0; i< track.motornum; i++)
            // {
            //     EC_WRITE_U8(domain[track.dm_index].domain_pd + track.motor[i].offset.mode_operation, track.motor[i].mode);
            // }
            // ready = changeTrackMotorState(track, 0, SWITCHED_ON);      // 临时为0

            // 在绝对编码器参与控制时写入期望位置才能使能电机
            // for (i = 0; i< 7; i++)
            // {
            //     EC_WRITE_S32(domain[leftarm.dm_index].domain_pd + leftarm.motor[i].offset.target_position, int(leftarm.motor[i].start_pos));
            // }

            if (ready)
            {
                run_state = CONTROL;
                printf("Elmo Mode Configuration Finished.\n");
            }
            break;

        case ENABLE:                // 不一定默认使能电机，所以不需要直接进入使能状态,TODO 将来需要默认使能履带速度电机
            ready = 1;
            for (i = 0; i < 7; i++) // 临时为1 sizeof(leftarm.motor)/sizeof(Motor)
            {
                if (leftarm.motor_use[i] == 1){
                    ready &= changeOneMotorState(leftarm, i, OPERATION_ENABLE);
                    printf("%d\n",i);
                }
                leftarm.motor[i].servo_cmd = 1;
            }
            // ready = changeTrackMotorState(track, 0, SWITCHED_ON);      // 临时为0

            if (ready)
            {
                printf("Elmo Enable Finished.\n");
                run_state = CONTROL;
            }
            break;

        case CONTROL:

            /********************** 读取当前位置 更新结构体 **********************/
            readArmData(leftarm);
            readArmData(rightarm);
            // readChassisData(leg, track);
            
            /********************** 接受并解析指令 根据指令进行控制 **********************/
            cmd = robotReceiveCommand();

            switch (cmd.cmd_mode)
            {
            
            case STOP_MOVE:
            {
                left_right = atoi(cmd.param_list[0]);
                if (left_right == LEFT)
                {
                    stopArmMotor(leftarm);
                }
                else if (left_right == RIGHT)
                {
                    stopArmMotor(rightarm);
                }
                else if (left_right == HEAD)
                {
                    stopArmMotor(head);
                }
                else if (left_right == LEG)
                {
                    stopArmMotor(leg);
                }
                
                break;
            }

            case ENABLE_MOTOR:
            {
                left_right = atoi(cmd.param_list[0]);
                enable_id = atoi(cmd.param_list[1]);
                if (enable_id < -1 || enable_id > 9)        // 非法数据
                {   
                    break;
                }
                if (left_right == LEFT)
                {
                    leftarm.state = IDLE;
                    if (enable_id == -1)
                    {
                        for ( i =0; i< leftarm.motornum; i++)
                        {
                            leftarm.motor[i].servo_cmd = 1;
                        }
                    }
                    else if (enable_id < leftarm.motornum )
                    {
                        printf("in");
                        leftarm.motor[enable_id].servo_cmd = 1;
                    }
                }
                else if (left_right == RIGHT)
                {
                    rightarm.state = IDLE;
                    if (enable_id == -1)
                    {
                        for ( i =0; i< rightarm.motornum; i++)
                        {
                            rightarm.motor[i].servo_cmd = 1;
                        }
                    }
                    else if (enable_id < rightarm.motornum )
                    {
                        rightarm.motor[enable_id].servo_cmd = 1;
                    }
                }
                else if (left_right == HEAD)
                {
                    head.state = IDLE;
                    if (enable_id == -1)
                    {
                        for ( i =0; i< head.motornum; i++)
                        {
                            head.motor[i].servo_cmd = 1;
                        }
                    }
                    else if (enable_id < head.motornum )
                    {
                        head.motor[enable_id].servo_cmd = 1;
                    }
                }
                else if ( left_right == LEG)
                {
                    leg.state = IDLE;
                    if (enable_id == -1)
                    {
                        for ( i =0; i< leg.motornum; i++)
                        {
                            leg.motor[i].servo_cmd = 1;
                        }
                    }
                    else if (enable_id < leg.motornum )
                    {
                        leg.motor[enable_id].servo_cmd = 1;
                    }
                }
                break;
            }

            case DISABLE_MOTOR:
            {
                left_right = atoi(cmd.param_list[0]);
                enable_id = atoi(cmd.param_list[1]);
                if (enable_id < -1 || enable_id > 9)        // 非法数据
                {   
                    break;
                }
                if (left_right == LEFT)
                {
                    if (enable_id == -1)
                    {
                        leftarm.state = DISABLE;
                        for ( i = 0; i< leftarm.motornum ;i ++)
                        {
                            leftarm.motor[i].servo_cmd = 0;
                        }
                    }
                    else if (enable_id < leftarm.motornum )
                    {
                        leftarm.motor[enable_id].servo_cmd = 0;
                    }
                }
                else if (left_right == RIGHT)
                {
                    if (enable_id == -1)
                    {
                        rightarm.state = DISABLE;
                        for ( i = 0; i< leftarm.motornum ;i ++)
                        {
                            leftarm.motor[i].servo_cmd = 0;
                        }
                    }
                    else if (enable_id < rightarm.motornum )
                    {
                        rightarm.motor[enable_id].servo_cmd = 0;
                    }
                }
                else if (left_right == HEAD)
                {
                    
                    if (enable_id == -1)
                    {
                        head.state = DISABLE;
                        for ( i = 0; i< leftarm.motornum ;i ++)
                        {
                            leftarm.motor[i].servo_cmd = 0;
                        }
                    }
                    else if (enable_id < head.motornum )
                    {
                        head.motor[enable_id].servo_cmd = 0;
                    }
                }
                else if ( left_right == LEG)
                {
                    if (enable_id == -1)
                    {
                        leg.state = DISABLE;
                        for ( i = 0; i< leftarm.motornum ;i ++)
                        {
                            leftarm.motor[i].servo_cmd = 0;
                        }
                    }
                    else if (enable_id < leg.motornum )
                    {
                        leg.motor[enable_id].servo_cmd = 0;
                    }
                }
                break;
            }

            case MOVEJ:
            {
                left_right = atoi(cmd.param_list[0]);
                speedRate = atof(cmd.param_list[cmd.param_cnt - 1]);
                
                if (cmd.param_cnt == 9)     // 给定关节角
                {
                    for (i = 0;i < 7; i++)
                    {
                        jointFinal[i] = atof(cmd.param_list[i + 1]);
                    }
                }
                else if (cmd.param_cnt == 8)        // 给定位姿
                {
                    for (i = 0;i < 6; i++)
                    {
                        poseFinal[i] = atof(cmd.param_list[i + 1]);
                    }
                    TfromPose(poseFinal, Tfinal);
                    /* 如果给定的是位姿 先求出位姿对应的关节角  静态大范围求解模式 β扫描间隔 0.01-3420us 0.1-520us*/
                    InverseKinematics(leftarm.jointPos, Tfinal, -M_PI, 0.1, M_PI, jointFinalBeta, angleFianl_beta_size);
                    
                    if (angleFianl_beta_size[1] == 8){      // 逆运动学成功
                        for (i = 0; i< 7; i++)
                        {
                            jointFinal[i] = jointFinalBeta[i];
                        }
                    }
                    else
                    {
                        printf("Inverse Kinematics Failed\n");
                    }
                }
                else if (cmd.param_cnt == 5)     // 给定头部关节角
                {
                    for (i = 0;i < 3; i++)
                    {
                        jointFinal[i] = atof(cmd.param_list[i + 1]);
                    }
                }
                else{
                    break;
                }
                
                if (left_right == LEFT)
                {
                    if (leftarm.state == IDLE)
                    {
                        printf_d(leftarm.jointPos,7);
                        printf_d(jointFinal, 7);
                        printf("Busy:in movej\n");
                        moveJ(leftarm, jointFinal, speedRate);
                    }
                }
                else if (left_right == RIGHT)
                {
                    if (rightarm.state == IDLE)
                    {
                        moveJ(rightarm, jointFinal, speedRate);
                    }
                }
                else if (left_right == HEAD)
                {
                    if (head.state == IDLE)
                    {
                        moveJ(head, jointFinal, speedRate);
                    }
                }
                else if (left_right == LEG)
                {
                    if (leg.state == IDLE)
                    {
                        moveJ(leg, jointFinal, speedRate);
                    }
                }
                
                // printf("%f,%f\n",leftarm.motor[0].sp.para[0],leftarm.motor[0].sp.para[1]);
                break;
            }

            case MOVEL:
            {
                printf("in moveL\n");
                left_right = atoi(cmd.param_list[0]);
                speedRate = atof(cmd.param_list[cmd.param_cnt - 1]);
                if (cmd.param_cnt == 8)     // 给定关节角
                {
                    for (i = 0;i < 6; i++)
                    {
                        poseFinal[i] = atof(cmd.param_list[i + 1]);
                    }
                    TfromPose(poseFinal, Tfinal);
                }
                if (left_right == LEFT)
                {
                    if (leftarm.state == IDLE)
                    {
                        moveLPoseChanged(leftarm, Tfinal, speedRate);       // 22423us 0.1-19520us 去掉findbeta 521us
                    }
                }
                else if (left_right == RIGHT)
                {
                    if (rightarm.state == IDLE)
                    {
                        moveLPoseChanged(rightarm, Tfinal, speedRate);
                    }
                }
                
                break;
            }

            case MOVE_FOLLOW:    
            {   
                left_right = atoi(cmd.param_list[0]);
                if (cmd.param_cnt == 8)     // 给定关节角
                {
                    for (i = 0;i < 7; i++)
                    {
                        jointFinal[i] = atof(cmd.param_list[i + 1]);
                    }
                }
                else if (cmd.param_cnt == 4)     // 给定关节角
                {
                    for (i = 0;i < 3; i++)
                    {
                        jointFinal[i] = atof(cmd.param_list[i + 1]);
                    }
                }
                else
                {
                    break;
                }
                
                if (left_right == LEFT)
                {
                    for (i = 0; i < 7; i++)
                    {
                        leftarm.motor[i].ref_position = jointFinal[i] * leftarm.jointGear[i];
                    }
                    leftarm.state = ON_MOVE_FOLLOW;
                }
                else if (left_right == RIGHT)
                {
                    for (i = 0; i < 7; i++)
                    {
                        rightarm.motor[i].ref_position = jointFinal[i] * rightarm.jointGear[i];
                    }
                    rightarm.state = ON_MOVE_FOLLOW;
                }
                else if (left_right == HEAD)
                {
                    for (i = 0; i < 3; i++)
                    {
                        head.motor[i].ref_position = jointFinal[i] * head.jointGear[i];
                    }
                    rightarm.state = ON_MOVE_FOLLOW;
                }
                break;
            }

            case CAR_MOVE:
            {
                if (cmd.param_cnt == 2)     // 给定运行速度 mm/s rad/s
                {
                    track.chassisVel_cmd[0] = atof(cmd.param_list[0]);
                    track.chassisVel_cmd[1] = atof(cmd.param_list[1]);
                    track.state = ON_CARMOVE;
                }
                break;
            }
            
            case HEADL:     //TODO!!!!!!!!!!!!
            {
                if (cmd.param_cnt == 3)     // 给定关节角
                {
                    for (i = 0;i < 6; i++)
                    {
                        if (i < 3)
                        {
                            jointFinal[i] = 0.0;
                        }
                        else
                        {
                            poseFinal[i] = atof(cmd.param_list[i - 3]);
                        }
                    }
                    TfromPose(poseFinal, Tfinal);

                }
                else
                {
                    break;
                }
                head.motor[0].exp_position = atoi(cmd.param_list[0]);
                head.motor[1].exp_position = atoi(cmd.param_list[1]);
                head.motor[2].exp_position = atoi(cmd.param_list[2]);
                printf("%f,%f,%f\n", head.motor[0].exp_position, head.motor[1].exp_position, head.motor[2].exp_position);
                break;
            }

            case FORCE_ENABLE:
            {
                left_right = atoi(cmd.param_list[0]);

                if (left_right == LEFT)
                {
                    if (leftarm.fctrl.Switch != 0)
                    {
                        break;
                    }
                    if (cmd.param_cnt == 1)     // 默认参数
                    {
                        memcpy(leftarm.fctrl.paramM, defaultM, sizeof(defaultM));
                        memcpy(leftarm.fctrl.paramK, defaultK, sizeof(defaultK));
                        for (i = 0; i<6; i++)
                        {
                            leftarm.fctrl.paramC[i] = 2.0 * defaultEP[i] * sqrt(leftarm.fctrl.paramK[i] * leftarm.fctrl.paramM[i]);
                        }
                    }
                    else if (cmd.param_cnt == 19)     // 给定参数
                    {
                        for (i = 0;i < 6; i++)
                        {
                            force_param = atof(cmd.param_list[i + 1]);        // 接收 M
                            if (force_param != 0.0)
                            {
                                leftarm.fctrl.paramM[i] = force_param;
                            }
                        }
                        for (i = 0;i < 6; i++)
                        {
                            force_param = atof(cmd.param_list[i + 1 + 6]);        // 接收 K
                            if (force_param != 0.0)
                            {
                                leftarm.fctrl.paramK[i] = force_param;
                            }
                        }
                        for (i = 0;i < 6; i++)
                        {
                            force_param = atof(cmd.param_list[i + 1 + 12]);       // 接收 epsilon
                            if (force_param != 0.0)
                            {
                                leftarm.fctrl.paramC[i] = 2 * force_param * sqrt( leftarm.fctrl.paramK[i] * leftarm.fctrl.paramM[i]);
                            }
                        }
                    }
                    leftarm.fctrl.Switch = -2;      // 第一周期需要清除力传感器，力控准备
                    memset(leftarm.fctrl.totalP, 0, sizeof(leftarm.fctrl.totalP));
                    memset(leftarm.fctrl.totalV, 0, sizeof(leftarm.fctrl.totalV));
                    memset(leftarm.fctrl.jointP, 0, sizeof(leftarm.fctrl.jointP));
                    memset(leftarm.fctrl.totalTrans, 0, sizeof(leftarm.fctrl.totalTrans));
                    leftarm.fctrl.totalTrans[0] = 1.0;
                    leftarm.fctrl.totalTrans[5] = 1.0;
                    leftarm.fctrl.totalTrans[10] = 1.0;
                    leftarm.fctrl.totalTrans[15] = 1.0;

                    clearForceSensor(leftarm.endft);     // 清零力传感器
                }
                else if (left_right == RIGHT)
                {
                    if (rightarm.fctrl.Switch != 0)     // 在停止状态下，开启
                    {
                        break;
                    }
                    if (cmd.param_cnt == 1)     // 默认参数
                    {
                        memcpy(rightarm.fctrl.paramM, defaultM, sizeof(defaultM));
                        memcpy(rightarm.fctrl.paramK, defaultK, sizeof(defaultK));
                        for (i = 0; i<6; i++)
                        {
                            rightarm.fctrl.paramC[i] = 2.0 * defaultEP[i] * sqrt(rightarm.fctrl.paramK[i] * rightarm.fctrl.paramM[i]);
                        }
                        
                    }
                    else if (cmd.param_cnt == 19)     // 给定关节角
                    {
                        for (i = 0;i < 6; i++)
                        {
                            force_param = atof(cmd.param_list[i + 1]);        // 接收 M
                            if (force_param != 0.0)
                            {
                                rightarm.fctrl.paramM[i] = force_param;
                            }
                        }
                        for (i = 0;i < 6; i++)
                        {
                            force_param = atof(cmd.param_list[i + 1 + 6]);        // 接收 K
                            if (force_param != 0.0)
                            {
                                rightarm.fctrl.paramK[i] = force_param;
                            }
                        }
                        for (i = 0;i < 6; i++)
                        {
                            force_param = atof(cmd.param_list[i + 1 + 12]);       // 接收 epsilon
                            if (force_param != 0.0)
                            {
                                rightarm.fctrl.paramC[i] = 2 * force_param * sqrt( rightarm.fctrl.paramK[i] * rightarm.fctrl.paramM[i]);
                            }
                        }
                    }
                    rightarm.fctrl.Switch = -2;      // 第一周期需要清除力传感器，力控准备
                    memset(rightarm.fctrl.totalP, 0, sizeof(rightarm.fctrl.totalP));
                    memset(rightarm.fctrl.totalV, 0, sizeof(rightarm.fctrl.totalV));
                    memset(rightarm.fctrl.jointP, 0, sizeof(rightarm.fctrl.jointP));
                    memset(rightarm.fctrl.totalTrans, 0, sizeof(rightarm.fctrl.totalTrans));
                    rightarm.fctrl.totalTrans[0] = 1.0;
                    rightarm.fctrl.totalTrans[5] = 1.0;
                    rightarm.fctrl.totalTrans[10] = 1.0;
                    rightarm.fctrl.totalTrans[15] = 1.0;
                    clearForceSensor(rightarm.endft);     // 清零力传感器
                   
                }
                
                break;
            }

            case FORCE_DISABLE:
            {
                 left_right = atoi(cmd.param_list[0]);
                
                if (left_right == LEFT)
                {
                    if (leftarm.fctrl.Switch == 1)
                    {
                        printf("waiting for exit\n");
                        leftarm.fctrl.Switch = -1;
                    }
                }
                else if (left_right == RIGHT)
                {
                    if (rightarm.fctrl.Switch == 1)
                    {
                        printf("waiting for exit\n");
                        rightarm.fctrl.Switch = -1;
                    }
                }
                
                break;
            }
            
            case MOVE_TEST:
            {
                left_right = atoi(cmd.param_list[0]);
                if (cmd.param_cnt == 1)
                {
                    test_A = 90.0 * DEG2RAD;      // 单位 度
                    test_T = 5.0;       // 单位 秒， 四分之一周期
                }
                else if (cmd.param_cnt == 3)
                {
                    test_A = atof(cmd.param_list[1]) * DEG2RAD;
                    test_T = atof(cmd.param_list[2]);
                }

                if (left_right == LEFT)
                {
                    leftarm.test_A = test_A;
                    leftarm.test_T = test_T;
                    leftarm.state = ON_MOVETEST;
                    leftarm.fctrl.Switch = 0;
                }
                else if (left_right == RIGHT)
                {
                    rightarm.test_A = test_A;
                    rightarm.test_T = test_T;
                    rightarm.state = ON_MOVETEST;
                    rightarm.fctrl.Switch = 0;
                }
                break;
            }

            case NO_RECV:
            {
                break;
            }
            
            case ERROR_MATCH:

                break;
            }

            /********************** 遍历各身体部位进行控制 **********************/
            ctrlArmMotor(leftarm);       // 控制左臂电机运动
            ctrlArmMotor(rightarm);       // 控制右臂电机运动
            // ctrlheadMotor(head);         // 控制头部电机运动
            // ctrlTrackMotor(track);          // 控制底盘履带电机运动
            
            robotSendFeedback(leftarm, rightarm, head, leg);

            break;

        case EXIT:

            break;
        }

        static time_t prev_second = 0;
        struct timeval tv;
        gettimeofday(&tv, 0);
        if (tv.tv_sec != prev_second)
        {
            // printf( "AKD: Loop time : %ldus\n",(long)period);
            prev_second = tv.tv_sec;
        }

        // send process data
        ecrt_domain_queue(domain[0].domain);
        ecrt_domain_queue(domain[1].domain);
        ecrt_master_send(master0);
        ecrt_master_send(master1);
        now = rt_timer_read();
        period = (now - previous) / 1000; //us
        // printf( "AKD: Loop time : %ldus\n",(long)period);

    }
    regfree(&reg);

    // 状态改变可以生效，但是Switched ON 状态不上伺服
    rt_task_wait_period(NULL);
    changeBodyMotorState(leftarm, -1, SWITCHED_ON);

    ecrt_domain_queue(domain[0].domain);
    ecrt_domain_queue(domain[1].domain);
    ecrt_master_send(master1);
    ecrt_master_send(master0);
    main_exit = 1;
}

/****************************************************************************
 * 信号量句柄
 ***************************************************************************/

void signal_handler(int sig)
{
    run = 0;
}

/****************************************************************************
 * 主函数
 ***************************************************************************/

int main(int argc, char *argv[])
{
    int ret;

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // ==================== 初始化插值数 ======================== //
    printf("Initializing interpolation param..\n");
    ip_param_init();

    // ==================== 配置UDP 通讯 ======================== //
    UDP_init();

    // ==================== 配置 EtherCAT 主站 ======================== //
    printf("Requesting master...\n");
    master0 = ecrt_request_master(0);
    if (!master0) return -1;

    master1 = ecrt_request_master(1);
    if (!master1) return -1;

    domain[0].domain = ecrt_master_create_domain(master0);
    if (!domain[0].domain) return -1;

    domain[1].domain = ecrt_master_create_domain(master1);
    if (!domain[1].domain) return -1;
    // ==================== 配置EtherCAT 从站 ======================== //
    printf("Creating slave configurations...\n");
    if (!leftarmInit(leftarm, master1, 1, left_slave_pos)) return -1;       // 配置左臂从站， 包括7个关节电机和1个力传感器, master0, domain0
    printf("leftarm init successed...\n");

    if (!rightarmInit(rightarm, master0, 0, right_slave_pos)) return -1;       // 配置右臂从站， 包括7个关节电机和1个力传感器, master0, domain0
    printf("rightarm init successed...\n");

    // if (!headInit(head, master0, 0, right_slave_pos)) return -1;       // 配置头从站， 包括3个关节电机,master0, domain0
    // printf("head init successed...\n");

    // if (!chassisInit(leg, track, master0, 0, track_slave_pos)) return -1;       // 配置头从站， 包括3个关节电机,master0, domain0
    // printf("chassis init successed...\n");
    
    // EC_position ft_pos = {0,0};
    // if (!FT_sensor_init(leftarm, master1, 1, ft_pos)) return -1;
    // printf("rightarm ftsensor init successed...\n");

    // ==================== 配置 EtherCAT PDO，激活主站，得到domain指针 ======================== //
    if (ArrayDomainRegs(domain[0]))
    {
        fprintf(stderr, "PDO domain0 entry registration failed!\n");
        return -1;
    }
    
    if (ArrayDomainRegs(domain[1]))
    {
        fprintf(stderr, "PDO domain1 entry registration failed!\n");
        return -1;
    }
  
    printf("Activating master...\n");
    // if (ecrt_master_activate(master0)) return -1;
    if (ecrt_master_activate(master1)) return -1;

    if (!(domain[0].domain_pd = ecrt_domain_data(domain[0].domain)))
    {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        return -1;
    }

    if (!(domain[1].domain_pd = ecrt_domain_data(domain[1].domain)))
    {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        return -1;
    }
   
    // ============================== 实时进程 ============================== //
    ret = rt_task_create(&my_task, "my_task", 0, 80, T_FPU);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to create task: %s\n", strerror(-ret));
        return -1;
    }

    printf("Starting my_task...\n");
    ret = rt_task_start(&my_task, &realtime_proc, NULL);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to start task: %s\n", strerror(-ret));
        return -1;
    }

    // ============================== 主函数循环 ============================== //

    while (!main_exit)
    {
        //nsleep(2000000);
        sched_yield();
    }

    // ============================== 退出程序 ============================== //

    printf("Deleting realtime task...\n");
    rt_task_delete(&my_task);

    printf("End of Program\n");
    ecrt_release_master(master1);
    ecrt_release_master(master0);

    return 0;
}
