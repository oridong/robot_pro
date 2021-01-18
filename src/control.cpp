
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

# define DUALARM -2
# define ALL -1
# define LEFT 0
# define RIGHT 1
# define HEAD 2
# define LEG 3
# define TRACK 4

# define LEFT_ETHERCAT master[0], 0
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

#define R1 310.0       // mm 单位 远端半径
#define R2 190.0       // mm 单位 近端半径
#define radius 130.0       // mm 单位

/***************************************************************
 * 变量及参数， 注意：电机参数列表需要大于使用电机数量
/**************************************************************/

// 使用电机或ethercat与否，调试需求
#define ETHERCAT_MAX 4
int ethercat_use[ETHERCAT_MAX] = {1, 0, 0, 0};
int bodypart_use[5] = {1, 0,  0, 0, 0};
int leftarm_use_motor[8] = {1, 0, 0, 0, 0, 0, 0, 1};

// EtherCAT 电机总线地址
static EC_position left_slave_pos[] = {{0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {0, 7}}; 

// 每个电机的减速比参数，编码器cnt/输出角度弧度值
const double leftarmGear[7] = {160.0 * 20480 * 2.0 / PI, 160.0 * 20480 * 2.0 / PI, 100.0 * 20480 * 2.0 / PI, 100.0 * 20480 * 2.0 / PI, 100.0 * 18000 * 2.0 / PI, 200 * 32768 * 2.0 / PI, 200 * 32768 * 2.0 / PI};

// 每个关节电机的机械减速比，用于抱闸解除试动
const double leftarmGearRatio[7] = {160.0, 160.0, 100.0, 100.0, 100.0, 200.0, 200.0};

// 各身体部分的工作模式
const uint8_t armMotorMode = 8; // 机械臂电机运行模式

// 插值周期
const uint8_t armMotoritpTimes[] = {10, 10, 10, 10, 10, 10, 10}; // 在初始化时对电机设置，能够对不同电机进行不同的设置, 默认插值倍数（插值周期与控制周期比）为10

// 每个关节电机的安装位置偏置//5.9651863174665865
const double leftoffsetAngle[7] = {5.179788154069138, 0.123045712266, 1.43972209997, 0.594982742005, 4.422179, 3.798304, 0.882806};        // 单位弧度

// 解决相对编码器和绝对编码器的不同invert问题，同向为1，不同向为-1
const int leftAbsRelRelation[7] = {1, 1, 1, 1, -1, -1, -1};

// 相对编码器方向
const int leftjointDir[7] = {-1, 1, -1, 1, 1, -1, 1};

// 抱闸使用
const uint8_t leftarm_brake[7] = {0, 0, 0, 0, 0, 0, 0};

// 绝对编码器的一圈的cnt，用于计算初始角度
const double leftAbsEncCnt[7] = {524288.0, 524288.0, 524288.0, 524288.0, 262144.0, 262144.0, 262144.0};

// 关节限位参数，up为最大值， down为最小值
double leftarmUpLimit[7] = {2.967, 2.094, 2.967, 2.094, 2.967, 2.181, 2.967};
double leftarmDownLimit[7] = {-2.967, -2.094, -2.967, -2.792, -2.967, -2.181, -2.967};

// 关节速度限制
double leftspeedlimit[7] = {PI/2, PI/2, PI/2, PI/2, PI/2, PI/2, PI/2};

// 力控默认参数
double defaultM[6] = {100, 100, 50, 5, 5, 5};
double defaultEP[6] = {1.1, 1.1, 1.1, 1.1, 1.1, 1.1};				/*阻尼比*/
double defaultK[6] = {150, 150, 200, 3, 3, 3};			/*刚度A6D_wn.^2.*A6D_m*/

// kdm fillter
double leftarm_fillter_k[7] = {500, 500, 500, 500, 500, 500, 500};
double leftarm_fillter_m[7] = {1, 1, 1, 1, 1, 1, 1};
double leftarm_fillter_esp[7] = {1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4};
double leftarm_fillter_vlimit[7] = {PI / 5, PI / 5, PI / 5, PI / 5, PI / 5, PI / 5, PI / 5};
double leftarm_fillter_acclimit[7] = {3 * PI, 3 * PI, 3 * PI, 3 * PI, 3 * PI, 3 * PI, 3 * PI};

float left_CL[7] = {11.0, 11.0, 7.0, 7.0, 5.0, 3.0, 3.0};
float left_PL[7] = {15.0, 15.0, 12.0, 12.0, 8.0, 5.0, 5.0};

// 变量声明
bodypart leftarm;

double alpha[itp_window] = {0.0f};
double alpha_sum = 0.0f;
std::vector<double> fil[100];
RT_TASK my_task;
int count = 0;

FILE *fp;

/************************************************
 * 重要状态位
 * *********************************************/
static int run = 1;
static int main_exit = 0;

/************************************************
 * EtherCat 相关结构体
 * *********************************************/
static ec_master_t *master[ETHERCAT_MAX] ={NULL, NULL, NULL, NULL};
static ec_master_state_t master_state[ETHERCAT_MAX] = {{}};

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
    {0x2203, 0x00, 32}, /* Analog Input*/
    {0x6078, 0x00, 16}, /* current actual value*/
    {0x6079, 0x00, 32}, /* voltage actual value*/
    {0x6062, 0x00, 32}  /* Target demond position*/
};

ec_pdo_info_t sp_pos[] = {  /* slave_pdos */
    {0x1600, 3, spe_pos + 0}, /* RPDO1 Mapping */
    {0x160B, 1, spe_pos + 3}, /* RPDO1 Mapping */
    {0x1a00, 3, spe_pos + 4}, /* TPDO1 Mapping */
    {0x1A24, 1, spe_pos + 7}, /* TPDO1 Mapping */
    {0x1A1F, 1, spe_pos + 8}, /* TPDO1 Mapping */
    {0x1A18, 1, spe_pos + 9},  /* TPDO1 Mapping */
    {0x1A0C, 1, spe_pos + 10}  /* TPDO1 Mapping */
};

ec_sync_info_t ss_pos[] = { /* slave_sync */ 
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, sp_pos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 5, sp_pos + 2, EC_WD_DISABLE},
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

void rt_check_master_state(ec_master_t *m, ec_master_state_t &result)
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

    memcpy(&result, &ms, sizeof(ec_master_state_t));
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
    arm.itp_period_times = 10;
    arm.movefollowCnt = 0;
    arm.motornum = 7;
    arm.state = DISABLE;
    arm.fctrl.Switch = 0;
    arm.teachEn = 0;

    uint8_t addr = 0;

    for (i = 0; i < arm.motornum; i++)     // ！！！！！！！！！ 只初始化一个电机作为实验 
    {
        arm.jointGear[i] = leftarmGear[i];
        arm.gearRatio[i] = leftarmGearRatio[i];
        arm.startJointAngle[i] = 0.0;
        arm.jointPos[i] = 0.0;
        arm.test_T = 5.0;
        arm.test_A = 90.0 * DEG2RAD;
        arm.motor_use[i] = leftarm_use_motor[i];
        arm.dir[i] = leftjointDir[i];

        arm.motor[i].first_time = 0;
        arm.motor[i].exp_position = 0;
        arm.motor[i].act_position = 0;
        arm.motor[i].start_pos = 0;
        arm.motor[i].mode = armMotorMode;
        arm.motor[i].this_send = 0;
        arm.motor[i].itp_period_times = armMotoritpTimes[i];
        arm.motor[i].plan_cnt = 0;
        arm.motor[i].plan_run_time = 0.0f;
        arm.motor[i].kdm.k = leftarm_fillter_k[i];
        arm.motor[i].kdm.c = 2 * leftarm_fillter_esp[i] *sqrt(leftarm_fillter_k[i] * leftarm_fillter_m[i]);
        arm.motor[i].kdm.m = leftarm_fillter_m[i];
        arm.motor[i].kdm.vlimit = leftarm_fillter_vlimit[i] * arm.jointGear[i];
        arm.motor[i].kdm.acclimit = leftarm_fillter_acclimit[i] * arm.jointGear[i];
        arm.motor[i].servo_first = 1;
        arm.motor[i].use_brake = leftarm_brake[i];
        arm.motor[i].CL = left_CL[i];

        // 保护参数
        arm.uplimit[i] = leftarmUpLimit[i] * arm.jointGear[i];
        arm.downlimit[i] = leftarmDownLimit[i] * arm.jointGear[i];
        arm.speedlimit[i] = leftspeedlimit[i] * arm.jointGear[i];

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
            ec_pdo_entry_reg_t temp9 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[8].index, spe_pos[8].subindex, &arm.motor[i].offset.current, NULL};
            ec_pdo_entry_reg_t temp10 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[9].index, spe_pos[9].subindex, &arm.motor[i].offset.voltage, NULL};
            ec_pdo_entry_reg_t temp11 = {arm.motor[i].alias, arm.motor[i].buspos, ELMO_GOLD, spe_pos[10].index, spe_pos[10].subindex, &arm.motor[i].offset.demond_position, NULL};

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
            domain[dm_index].domain_reg.push_back(temp11);

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

            // 保护性电流
            uint8_t *data2 = (uint8_t *)&(left_CL[i]);
            data_size = sizeof(left_CL[i]);
            if (ecrt_master_sdo_download(m, arm.motor[i].buspos, 0x303f, 1, data2, data_size, &abort_code)) // 写SDO，0x2F41:0 16-19位配置为4 即可开启模拟二通道输入
            {
                fprintf(stderr, "cl change failed.\n");
            }

            // 峰值电流
            uint8_t *data4 = (uint8_t *)&(left_PL[i]);
            data_size = sizeof(left_PL[i]);
            if (ecrt_master_sdo_download(m, arm.motor[i].buspos, 0x3191, 1, data4, data_size, &abort_code)) // 写SDO，0x2F41:0 16-19位配置为4 即可开启模拟二通道输入
            {
                fprintf(stderr, "pl change failed.\n");
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

            arm.startJointAngle[i] = *((int32_t *)result) / leftAbsEncCnt[i] * 2 * PI;  // ()/262144 * 2 * -PI
            arm.startJointAngle[i] -= leftoffsetAngle[i];
           
            arm.startJointAngle[i] = arm.startJointAngle[i] * double(leftjointDir[i] * leftAbsRelRelation[i]);

            arm.startJointAngle[i] -= floor((arm.startJointAngle[i] + PI)/(2 * PI)) * 2 * PI;
            
            printf("sdo config OK.%d,abs pos:%f rad,rel pos:%d\n", i, arm.startJointAngle[i], arm.motor[i].start_pos);
        }

    } /* motor 循环*/

    ecrt_slave_config_dc(arm.motor[0].sc_dig_out, 0x700, 1000000, 0, 0, 0);

    // 初始化力传感器从站
    arm.motor_use[7] = leftarm_use_motor[7];
    if (leftarm_use_motor[7] == 1){
        EC_position ft_pos = {motor_pos[addr].alias, motor_pos[addr].buspos};
        i = FT_sensor_init(arm, m, dm_index, ft_pos);
    }
    
    return i;
    // return 1;
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
    
    // 4、从0x6000:0x03 读Sdo 试读力传感器数据Fz
    // if (ecrt_master_sdo_upload(m, pos.buspos, 0x6000, 0x02, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
    // {
    //     fprintf(stderr, "Failed to get sdo data.\n");
    //     return -1;
    // }
    // data1 = result[3] << 24 | result[2] << 16 | result[1] << 8 | result[0];
    // printf("%f\n",(float)data1/1000000.0);

    return 1;
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
 * 二阶系统滤波器 位置电机
 */
void kdmfillter(Motor &m)
{
    double acc;
    double dt;
    double deltaX;
    dt = (double)ctl_period / 1e9;
    deltaX = (double)m.exp_position_kdm - m.exp_position;
    
    acc = (-m.kdm.k * deltaX - m.kdm.c * m.exp_position_kdm_v)/m.kdm.m;

    if (acc > m.kdm.acclimit)
    {
        acc = m.kdm.acclimit;
    }
    else if(acc < -m.kdm.acclimit)
    {
        acc = -m.kdm.acclimit;
    }

    m.exp_position_kdm_v += acc * dt; 
    
    if (m.exp_position_kdm_v> m.kdm.vlimit)
    {
        m.exp_position_kdm_v= m.kdm.vlimit;
    }
    else if (m.exp_position_kdm_v< -m.kdm.vlimit)
    {
        m.exp_position_kdm_v= -m.kdm.vlimit;
    }

    m.exp_position_kdm += m.exp_position_kdm_v * dt + 0.5 * acc * dt *dt;

    // printf("%f, %f\n",m.exp_position_kdm, m.exp_position);
}

/*
 * 中值滤波器
 */
double midvfillter(double data, int index, int size)
{
    double sum;
    int i;
    sum = 0.0;
    while(fil[index].size() <= size)
    {
        fil[index].push_back(data);
    }

    if (fil[index].size() > size)
    {
        fil[index].erase(fil[index].begin());
        for (i = 0; i< fil[index].size(); i++)
        {
            sum += fil[index].at(i)/(double)size;
        }
        return sum;
    }
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
        m.plan.push_back(m.exp_position_kdm);
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
        printf("%d in Fault state\n", id);
        arm.state = ERROR;
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
        arm.motor[i].exp_position_kdm = arm.motor[i].act_position;
        arm.motor[i].exp_position_kdm_v = 0.0;
        arm.motor[i].ref_position = arm.motor[i].act_position;
        arm.motor[i].plan_cnt = 0;
        // arm.motor[i].plan.clear();
    }
    arm.state = IDLE;
}

/*
 * 读手臂电机反馈数据，输入左臂或右臂
 */
void readArmData(bodypart & arm)
{
    int i;
    int motornum = arm.motornum;
    int vol_rec = 0;
    for (i = 0, vol_rec = 0; i < motornum; i++)
    {
        if (arm.motor_use[i] == 1)
        {
            arm.motor[i].act_position = (int)arm.dir[i] * (EC_READ_S32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.act_position) - arm.motor[i].start_pos) + int((arm.startJointAngle[i]) * arm.jointGear[i]);
            arm.motor[i].act_current = (double)EC_READ_S16(domain[arm.dm_index].domain_pd + arm.motor[i].offset.current)/1000.0 * arm.motor[i].CL;
            arm.motor[i].ain = EC_READ_U32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.ain);
            // rt_printf("id:%d, ad:%d\n", i, arm.motor[i].ain);
            arm.motor[i].act_voltage = (double)EC_READ_U32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.voltage)/1000.0;
            // arm.motor[i].demond_position = EC_READ_S32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.target_position);
            arm.motor[i].demond_position = EC_READ_S32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.demond_position);

            if (vol_rec == 0){
                arm.act_voltage = arm.motor[i].act_voltage;
                vol_rec = 1;
            }
        }
        else
        {
            arm.motor[i].act_position = 0;
            arm.motor[i].act_current = 0;
            arm.motor[i].ain = 0;
            arm.act_voltage = 0;
        }

        
        // printf("%d\n", arm.motor[i].act_position);
        if (arm.motor[i].first_time == 0) // 初次进入，记录开机时刻位置作为期望位置
        {
            printf("first:%d, act_position:%.2f degree\n", i, arm.motor[i].act_position / arm.jointGear[i] * RAD2DEG);
            arm.motor[i].exp_position = arm.motor[i].act_position;
            arm.motor[i].ref_position = arm.motor[i].act_position;
            arm.motor[i].exp_position_kdm = arm.motor[i].act_position;
            arm.motor[i].last_actposition = arm.motor[i].act_position;
            arm.motor[i].first_time = 1;
        }
        if (arm.motor[i].servo_state == 1 || arm.teachEn == 1)
        {
            arm.jointPos[i] = (double)(arm.motor[i].act_position) / arm.jointGear[i] ;
        }
        else if (arm.motor[i].servo_state == 0)
        {
            arm.jointPos[i] = (double)(arm.motor[i].exp_position_kdm) / arm.jointGear[i] ;
        }

        // printf("%f\n", arm.jointPos[i]);

    }
    // printf("%f\n", arm.motor[0].ref_position);
    // printf("%f, %f, %f, %f, %f, %f, %f\n", arm.motor[0].act_position, arm.motor[1].act_position, arm.motor[2].act_position, arm.motor[3].act_position, arm.motor[4].act_position, arm.motor[5].act_position,  arm.motor[6].act_position);
}

/*
 * 读力传感器数据，输入左臂或右臂
 */
void readForceData(bodypart &arm)
{
    int dm_index = arm.dm_index;
    int i = 0;
    double buf[6];
    if (arm.motor_use[7] == 1){
        buf[0] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Fx)/arm.endft.countsPerForce - arm.endft.offsetft[0];
        buf[1] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Fy)/arm.endft.countsPerForce - arm.endft.offsetft[1];
        buf[2] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Fz)/arm.endft.countsPerForce - arm.endft.offsetft[2];
        buf[3] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Tx)/arm.endft.countsPerTorque - arm.endft.offsetft[3];
        buf[4] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Ty)/arm.endft.countsPerTorque - arm.endft.offsetft[4];
        buf[5] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Tz)/arm.endft.countsPerTorque - arm.endft.offsetft[5];
        for (i = 0; i< 6; i++)
        {
            arm.endft.ft[i] = midvfillter(buf[i], i + 2 + 7 * arm.dm_index, 200);
        }

        arm.endft.status = EC_READ_U32(domain[dm_index].domain_pd + arm.endft.offset.status1);
        arm.endft.sampCount = EC_READ_U32(domain[dm_index].domain_pd + arm.endft.offset.status2);
    }

    // arm.endft.ft[3] = 2.0; // !!!!!!!!!!!!!!!!!!!
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
 * 读底盘反馈数据
 */
void readChassisData(bodypart & leg, trackpart & trc)
{
    int i;
    for (i = 0; i < leg.motornum; i++)
    {
        leg.motor[i].act_position = EC_READ_S32(domain[leg.dm_index].domain_pd + leg.motor[i].offset.act_position) - leg.motor[i].start_pos + (int)(leg.startJointAngle[i] * leg.jointGear[i]);
       
        leg.motor[i].act_current = (double)EC_READ_S16(domain[leg.dm_index].domain_pd + leg.motor[i].offset.current)/1000.0 * leg.motor[i].CL;
        leg.motor[i].ain = EC_READ_U32(domain[leg.dm_index].domain_pd + leg.motor[i].offset.ain);
        if (leg.motor[i].first_time == 0) // 初次进入，记录开机时刻位置作为期望位置
        {
            printf("first:%d, act_position:%d\n", i, leg.motor[i].act_position);
            leg.motor[i].exp_position = leg.motor[i].act_position;
            leg.motor[i].exp_position_kdm = leg.motor[i].act_position;
            leg.motor[i].exp_position_kdm_v = 0;
            leg.motor[i].ref_position = leg.motor[i].act_position;
	    leg.motor[i].first_time = 1;
        }
        leg.jointPos[i] = (double)(leg.motor[i].act_position)/ leg.jointGear[i];
    }
    double a = 0.003158 * (midvfillter(leg.motor[0].ain, 1, 5000) - 2000.6);
	// printf("%f,%f\n",a, leg.jointPos[0]);
	// printf("%f,%f,%f,%f\n",leg.motor[0].act_current,leg.motor[1].act_current,leg.motor[2].act_current,leg.motor[3].act_current);

    for (i = 0; i < trc.motornum; i++)
    {
        trc.motor[i].act_velocity = EC_READ_S32(domain[trc.dm_index].domain_pd + trc.motor[i].offset.act_velocity);
        trc.motor[i].act_position = EC_READ_S32(domain[trc.dm_index].domain_pd + trc.motor[i].offset.act_position);

        trc.motor[i].act_current = (double)EC_READ_S16(domain[trc.dm_index].domain_pd + trc.motor[i].offset.current)/1000.0 * trc.motor[i].CL ;
        trc.motor[i].act_voltage = (double)EC_READ_U32(domain[trc.dm_index].domain_pd + trc.motor[i].offset.voltage)/1000.0 ;
        
        if (trc.motor[i].first_time == 0) // 初次进入，记录开机时刻位置作为期望位置
        {
            printf("first:%d, act_velcity:%d,actposition: %d\n", i, trc.motor[i].act_velocity, trc.motor[i].act_position);
            trc.motor[i].exp_velocity = 0;
            trc.motor[i].first_time = 1;
        }
        trc.jointVel[i] = (double)trc.motor[i].act_velocity / trc.jointGear[i];
    }
    // printf("%d,%d,%d,%d\n", trc.motor[0].act_velocity,  trc.motor[1].act_velocity,  trc.motor[2].act_velocity,  trc.motor[3].act_velocity);

}

void jointProtection(bodypart &arm)
{
    int i = 0;
    int motornum = arm.motornum;
    int protect_flag = 0;
    for (i = 0; i< motornum; i++)
    {
        if (arm.motor[i].exp_position > arm.uplimit[i])
        {
            arm.motor[i].exp_position = arm.uplimit[i];
        }
        else if (arm.motor[i].exp_position < arm.downlimit[i])
        {
            arm.motor[i].exp_position = arm.downlimit[i];
        }

        if (abs(arm.motor[i].act_position - arm.motor[i].last_actposition) / 0.001 > arm.speedlimit[i] * arm.jointGear[i])
        {
            printf("%d speed:%f,",i, abs(arm.motor[i].act_position - arm.motor[i].last_actposition) / 0.001/ arm.jointGear[i]);
            protect_flag = 1;
        }
    }
    if (protect_flag == 1)      // 速度超限幅
    {
        arm.state = IDLE;
        for (i = 0; i< motornum; i++)
        {
            arm.motor[i].ref_position = arm.motor[i].act_position;
            arm.motor[i].exp_position = arm.motor[i].act_position;
        }
        printf("detected out of speed limit!!\n");
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
    uint32_t brake_output;

    static uint16_t waittime = 0;
    
    switch (arm.state) // 根据不同功能得到motor.ref_position的值，进行不同种类的控制
    {
        case DISABLE:       // 整臂去使能状态 TODO 去使能先刹车
            for (i = 0; i< motornum; i++)
            {
                if (arm.motor[i].servo_state == 1)
                {
                    ret = changeOneMotorState(arm, i, SWITCHED_ON);
                    
                    if (ret){
                        arm.motor[i].servo_state = 0;
                        brake_output = 0x00000;
                        EC_WRITE_U32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.DO, brake_output);  // 去使能成功加抱闸
                    }
                }
            }
            arm.fctrl.Switch = 0;       //关闭力控
        break;

        case IDLE:      // 空闲状态，可以进行任务
            arm.test_time = 0.0;
            servoCmdAll = 0;
            servoStateAll = 0;
            for (i = 0; i< motornum; i++)
            {
                if (arm.motor_use[i] == 1)
                {
                    if (arm.motor[i].servo_cmd == 1)
                    {
                        if (arm.motor[i].ref_position > arm.uplimit[i] || arm.motor[i].ref_position < arm.downlimit[i])
                        {
                            if (arm.motor[i].servo_first == 1){
                                arm.motor[i].servo_cmd = 0;
                                arm.motor[i].servo_first = 0;
                                ret = 0;
                                printf("%d, out of limit, enable again\n", i);
                            }
                            else
                            {
                                ret = changeOneMotorState(arm, i, OPERATION_ENABLE);
                                // printf("%d, out of limit, still move\n", i);
                            }
                        }
                        else
                        {
                            ret = changeOneMotorState(arm, i, OPERATION_ENABLE);
                        }

                        if (ret)
                        {
                            if ( arm.motor[i].servo_state != 1){
                                
                                arm.motor[i].exp_position_kdm_v = 0;
                                arm.motor[i].exp_position_kdm = arm.motor[i].act_position;
                                if (i > 4){             // 67 关节
                                    arm.motor[i].ref_position = arm.motor[i].act_position;      // 清除遗留目标位置
                                    arm.motor[i].plan_cnt = 0;
                                    arm.motor[i].plan.clear();
                                    arm.motor[i].servo_state = 1;
                                }
                                else if (arm.motor[i].servo_state == 0)             // 1-5关节 第一次
                                {
                                    if (arm.motor[i].use_brake == 1)
                                    {
                                        brake_output = 0x10000;
                                        EC_WRITE_U32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.DO, brake_output);  // 上使能成功解除抱闸
                                        arm.motor[i].ref_position = arm.motor[i].act_position - 1.0 * DEG2RAD * arm.jointGear[i] / arm.gearRatio[i];  // 使用抱闸动作
                                        arm.motor[i].plan_cnt = 0;
                                        arm.motor[i].servo_state = 2; 
                                    }
                                    else
                                    {
                                        arm.motor[i].ref_position = arm.motor[i].act_position ;
                                        arm.motor[i].plan_cnt = 0;
                                        arm.motor[i].plan.clear(); 
                                        arm.motor[i].servo_state = 1;   
                                    }
                                }
                            }
                        }

                        // 解除抱闸动作
                        if (arm.motor[i].servo_state > 1)
                        {
                            arm.motor[i].servo_state ++;
                        }

                        if (arm.motor[i].servo_state == 200)
                        {
                            arm.motor[i].ref_position = arm.motor[i].act_position + 2.0 * DEG2RAD * arm.jointGear[i] / arm.gearRatio[i];
                            arm.motor[i].plan_cnt = 0;
                            printf("%d, %f\n", arm.motor[i].act_position, arm.motor[i].ref_position);
                        }

                        if (arm.motor[i].servo_state > 600)
                        {
                            arm.motor[i].servo_state = 1;
                        }
                    }
                    else if (arm.motor[i].servo_cmd == 0 && arm.motor[i].servo_state == 1)
                    {
                        ret = changeOneMotorState(arm, i, SWITCHED_ON);
                        if (ret){
                            arm.motor[i].servo_state = 0;
                            brake_output = 0x00000;
                            EC_WRITE_U32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.DO, brake_output);  // 去使能成功加抱闸
                        }
                    }
                }
                
                servoCmdAll |= arm.motor[i].servo_cmd;      // 检测全零
                servoStateAll |= arm.motor[i].servo_state;
            }
            if (servoCmdAll == 0 && servoStateAll == 0)
            {
                printf("no motor enabled, state to DISABLE\n");
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
                printf_d(location, 3);

                beta_line = S_position(arm.s_beta.time, arm.s_beta.para);
                equat_line[0] = arm.rEquivalent[0];//S_position(arm.s_equat.time, arm.s_equat.para);
                equat_line[1] = arm.rEquivalent[1];
                equat_line[2] = arm.rEquivalent[2];
                equat_line[3] = arm.rEquivalent[3];

                // 将当前位置与初始姿态组成位姿矩阵
                quat2rot(equat_line, rot_);
                matrixMultiply(rot_, 3, 3, arm.rotInit, 3, 3, R);
                TfromRotPos(R, location, pose_line);

                printf("new\n");
                printf_d(pose_line, 16);
                printf("betafind: %f\n", FindBeta(arm.jointPos));

                InverseKinematics(arm.jointPos, pose_line, beta_line-0.1, 0.01, beta_line+0.1, angle_planned, angle_planned_size);
                printf_d(angle_planned, 8);
                printf("%f,%f,%f,%f,%f,%f,%f  ", arm.jointPos[0], arm.jointPos[1], arm.jointPos[2], arm.jointPos[3], arm.jointPos[4], arm.jointPos[5], arm.jointPos[6]);
                printf("%f\n", beta_line);

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
                // printf("%f,%f,%f,%f,%f,%f,%f\n",angle_planned[0], angle_planned[1], angle_planned[2], angle_planned[3], angle_planned[4], angle_planned[5], angle_planned[6] );
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
                // arm.state = IDLE;       // 退出moveL的执行
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
                    // printf("%f,", arm.motor[i].ref_position);
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
            if (cos(arm.test_time * PI / arm.test_T) >= cos(0.0005)){
                waittime ++;
            }

            if (waittime == 0){
                arm.test_time += 0.001;
            }
            else if (waittime == 2000)
            {
                waittime = 0;
                arm.test_time += 0.001;
            }
            
            // printf("%d, %d, %f\n", cos(arm.test_time * PI / arm.test_T) == 1, waittime, arm.test_time);

            
        break;

        case ON_MOVE_FOLLOW:
            // 超时检测，在接收到新的moveFollow指令清除看门狗
            check_follow(arm, 0.5);
            break;

        default:
            break;
    }

    // 经过力控制的滤波器，对ref_position叠加一个力偏置得到期望电机position
    int dir_enalbe[6] = {1, 1, 1, 1, 1, 1};     // 力控制笛卡尔空间使能
    uint8_t forceCtrlType = 0;      // 0 逆运动学直接模式， 1 逆运动学差分模式（卡顿）， 2 雅克比模式（不稳定）

    // 力控状态机，由于没有状态切换之间的等待， 所以只使用命令字表示状态
    if (arm.fctrl.Switch == 1)      // 力控打开
    {
        forceUpdate(arm, forceCtrlType, 0.001, dir_enalbe);  // deltaT in 秒
    }
    else if (arm.fctrl.Switch == -1)        // 力控缓慢关闭，会强制回到跟随的位置
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
            printf("no force data,please check\n");
    }
    else if (arm.fctrl.Switch == 2)    // 力超出限制，力控快速关闭
    {
        for (i = 0; i< 6; i++)
        {
            arm.fctrl.totalP[i] = 0;
            arm.fctrl.totalV[i] = 0;
        }
        for (i = 0 ; i < motornum; i++)
        {
            arm.motor[i].ref_position = arm.motor[i].act_position;
            arm.motor[i].exp_position = arm.motor[i].act_position;
        }
        arm.fctrl.Switch = 0;
        printf("closed forceCtrl\n");
    }
    else if (arm.fctrl.Switch == 0)     // 力控关闭，参考位置直接进行插值
    {
        for (i = 0 ; i < motornum; i++)
        {
            arm.motor[i].exp_position = arm.motor[i].ref_position  + arm.test_A / 2.0 * leftarmGear[i] * (1 - cos(arm.test_time * PI / arm.test_T));
        }
    }

    // printf("%f\n",arm.motor[0].exp_position);
    //printf("%.2f,%.2f,%.2f\n", double(arm.motor[1].ref_position)/arm.jointGear[1]*RAD2DEG,  arm.motor[2].ref_position/arm.jointGear[2]*RAD2DEG, arm.motor[3].ref_position/ arm.jointGear[3] * RAD2DEG);

    // jointProtection(arm);

    // 电机遍历取值，精插补
    for (i = 0; i < motornum; i++)
    {
        kdmfillter(arm.motor[i]);
        /********************* 电机轨迹精插值规划 **********************/
        if (arm.motor[i].plan_cnt == 0)
        {
            interpolation(arm.motor[i]);
        }
        /********************* 电机轨迹精插值运动 **********************/
        arm.motor[i].plan_run_time = double(arm.motor[i].plan_cnt) * double(ctl_period) / 1e9;
        arm.motor[i].this_send = arm.motor[i].plan_param[0] + arm.motor[i].plan_param[1] * arm.motor[i].plan_run_time + arm.motor[i].plan_param[2] * arm.motor[i].plan_run_time * arm.motor[i].plan_run_time + arm.motor[i].plan_param[3] * arm.motor[i].plan_run_time * arm.motor[i].plan_run_time * arm.motor[i].plan_run_time;

        arm.motor[i].plan_cnt++;
        if (arm.motor[i].plan_cnt >= arm.motor[i].itp_period_times)
        {
            arm.motor[i].plan_cnt = 0;
        }

        if (arm.motor_use[i] == 1){
        /********************** 填写指令，等待发送 **********************/
            EC_WRITE_S32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.target_position,  (int)(arm.dir[i] * arm.motor[i].this_send) + arm.motor[i].start_pos - int(arm.dir[i] * arm.startJointAngle[i] * arm.jointGear[i]));
            // printf("%f, %f, %d\n",arm.jointPos[i], arm.motor[i].exp_position, (int)arm.motor[i].this_send + arm.motor[i].start_pos - int(arm.startJointAngle[i] * arm.jointGear[i]));
            
            if (i == 1){
                RTIME thistime;
                thistime = rt_timer_read();
                fprintf(fp, "%lf:target: %d, demond: %d, act: %d\n", thistime/1e9, (int)(arm.dir[i] * arm.motor[i].this_send) + arm.motor[i].start_pos - int(arm.dir[i] * arm.startJointAngle[i] * arm.jointGear[i]), arm.motor[i].demond_position, int((arm.motor[i].act_position - int((arm.startJointAngle[i]) * arm.jointGear[i])) * arm.dir[i]) + arm.motor[i].start_pos);
                
                count ++;
            }
        }

    }
    // fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", arm.motor[0].this_send/arm.jointGear[0], arm.motor[1].this_send/arm.jointGear[1], arm.motor[2].this_send/arm.jointGear[2], arm.motor[3].this_send/arm.jointGear[3], arm.motor[4].this_send/arm.jointGear[4], arm.motor[5].this_send/arm.jointGear[5],arm.motor[6].this_send/arm.jointGear[6],arm.endft.ft[0],arm.endft.ft[1],arm.endft.ft[2],arm.endft.ft[3],arm.endft.ft[4],arm.endft.ft[5]);
   
}


// 检查follow超时，timeout单位为秒
void check_follow(bodypart & arm, double timeout)
{
    arm.movefollowCnt ++;
    if (arm.movefollowCnt >= (int)(timeout / ((double)ctl_period /1e9 )) )
    {
        arm.movefollowCnt = 0;
        arm.state = IDLE;
        printf("leave movefollow state\n");
    }
}

/**
 * 实时任务函数
 */
void realtime_proc(void *arg)
{
    // 系统时间
    RTIME now, previous, period, start_time, totaltime, last_moment;
    start_time = rt_timer_read();

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

    // speedL
    double T07[16];
    double Tdelta[16];
    double delta[6];
    double speed[6];
    int inToolFrame = 0;
    double pose07[6];
    double jointrefpos[7];
    double beta_cmd;
    

    while (run)
    {
        rt_task_wait_period(NULL);      // 等待定时周期
        previous = rt_timer_read();     // 获取运行周期开始时间
        
        // 接收 EtherCAT 帧
        for ( i =0; i< ETHERCAT_MAX; i++)
        {
            if (ethercat_use[i] == 1)
            {
                ecrt_master_receive(master[i]);
                ecrt_domain_process(domain[i].domain);
            }
        }

        switch (run_state)
        {

        case CONFIG_ELMO:

            ready = 1;
            if (bodypart_use[LEFT]){
                for (i = 0; i < leftarm.motornum; i++)
                {
                    if (leftarm.motor_use[i] == 1){
                        EC_WRITE_U8(domain[leftarm.dm_index].domain_pd + leftarm.motor[i].offset.mode_operation, leftarm.motor[i].mode);
                        ready &= changeOneMotorState(leftarm, i, SWITCHED_ON);
                    }
                }
            }

            // 在绝对编码器参与控制时写入期望位置才能使能电机
            // for (i = 0; i< 7; i++)
            // {
            //     EC_WRITE_S32(domain[leftarm.dm_index].domain_pd + leftarm.motor[i].offset.target_position, int(leftarm.motor[i].start_pos));
            // }

            if (ready)
            {
                run_state = CONTROL;
                // printf("Elmo Mode Configuration Finished.\n");
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
            
            if (bodypart_use[LEFT]){
                readArmData(leftarm);
                readForceData(leftarm);
            }

            /********************** 接受并解析指令 根据指令进行控制 **********************/
            cmd = robotReceiveCommand();
            fprintf(fp, "cmd:%d\n", cmd.cmd_mode);

            switch (cmd.cmd_mode)
            {
            
            case STOP_MOVE:
            {

                if (!CM_Atoi(cmd.param_list[0], left_right))
                    break;
                if (left_right == ALL)
                {
                    stopArmMotor(leftarm);
                    stopArmMotor(rightarm);
                    stopArmMotor(head);
                    stopArmMotor(leg);
                    stopTrackMotor(track);
                }
                
                break;
            }

            case ENABLE_MOTOR:
            {
                if (!CM_Atoi(cmd.param_list[0], left_right))
                    break;
                if (!CM_Atoi(cmd.param_list[1], enable_id))
                    break;

                if (enable_id < -1 || enable_id > 9)        // 非法数据
                {   
                    break;
                }
                if (left_right == ALL){
                    if (enable_id == -1)
                    {
                        if (bodypart_use[LEFT]){
                            leftarm.state = IDLE;
                            for ( i =0; i< leftarm.motornum; i++)
                            {
                                leftarm.motor[i].servo_cmd = 1;
                            }
                        }
                    }
                }
                else if (left_right == LEFT)
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
                        // printf("in");
                        leftarm.motor[enable_id].servo_cmd = 1;
                    }
                }
                
                break;
            }

            case DISABLE_MOTOR:
            {
                if (!CM_Atoi(cmd.param_list[0], left_right))
                    break;
                if (!CM_Atoi(cmd.param_list[1], enable_id))
                    break;
                if (enable_id < -1 || enable_id > 9)        // 非法数据
                {   
                    break;
                }
                if (left_right == ALL){
                    if (enable_id == -1)
                    {
                        if (bodypart_use[LEFT]){
                            leftarm.state = DISABLE;
                            for ( i =0; i< leftarm.motornum; i++)
                            {
                                leftarm.motor[i].servo_cmd = 0;
                            }
                        }
                    }
                }
                else if (left_right == LEFT)
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
                break;
            }

            case MOVEJ:
            {
                if (!CM_Atoi(cmd.param_list[0], left_right))
                    break;
                if (!CM_Atof(cmd.param_list[cmd.param_cnt - 1], speedRate))
                    break;
                
                if (cmd.param_cnt == 9)     // 给定关节角
                {
                    for (i = 0;i < 7; i++)
                    {
                        if (!CM_Atof(cmd.param_list[i + 1], jointFinal[i]))
                            break;
                    }
                }
                else if (cmd.param_cnt == 8)        // 给定位姿
                {
                    for (i = 0;i < 6; i++)
                    {
                        if (!CM_Atof(cmd.param_list[i + 1], poseFinal[i]))
                            break;
                    }
                }
                else{
                    break;
                }
                
                if (left_right == LEFT)
                {
                    if (cmd.param_cnt == 8){
                        TfromPose(poseFinal, Tfinal);
                        /* 如果给定的是位姿 先求出位姿对应的关节角  静态大范围求解模式 β扫描间隔 0.01-3420us 0.1-520us*/
                        InverseKinematics(leftarm.jointPos, Tfinal, -M_PI, 0.1, M_PI, jointFinalBeta, angleFianl_beta_size);
                        // printf_d(jointFinalBeta, 8);
                        
                        if (angleFianl_beta_size[1] == 8){      // 逆运动学成功
                            for (i = 0; i< 7; i++)
                            {
                                jointFinal[i] = jointFinalBeta[i];
                            }
                        }
                        else
                        {
                            printf("Inverse Kinematics Failed\n");
                            break;
                        }
                    }

                    for (i = 0; i< leftarm.motornum; i++)
                    {
                        leftarm.motor[i].itp_period_times = 10;
                        printf("%f ", leftarm.motor[i].ref_position/leftarm.jointGear[i]);
                    }
                    leftarm.itp_period_times = 10;
                    // if (leftarm.state == IDLE)       // 仿真不上伺服moveJ
                    {
                        printf("Busy:in movej\n");
                        moveJ(leftarm, jointFinal, speedRate);
                    }
                }
               
                break;
            }

            case MOVE_FOLLOW:    
            {   
               if (!CM_Atoi(cmd.param_list[0], left_right))
                    break;
                if (cmd.param_cnt == 8 || cmd.param_cnt == 4 || cmd.param_cnt == 6)     // 给定关节角
                {
                    for (i = 0;i < cmd.param_cnt - 1; i++)
                    {
                        if (!CM_Atof(cmd.param_list[i + 1], jointFinal[i]))
                            break;  
                    }
                }
                else
                {
                    printf("InValid Input\n");
                    break;
                }
                
                if (left_right == LEFT)
                {
                    for (i = 0; i < 7; i++)
                    {
                        leftarm.motor[i].ref_position = jointFinal[i] * leftarm.jointGear[i];
                    }
                    leftarm.state = ON_MOVE_FOLLOW;
                    leftarm.movefollowCnt = 0;
                }
                break;
            }

            case FORCE_ENABLE:
            {
                if (!CM_Atoi(cmd.param_list[0], left_right))
                    break;

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
                            if (!CM_Atof(cmd.param_list[i + 1], force_param))
                                break;
                            if (!isnan(force_param) && !isinf(force_param))
                            {
                                leftarm.fctrl.paramM[i] = force_param;
                            }
                        }
                        for (i = 0;i < 6; i++)
                        {
                            if (!CM_Atof(cmd.param_list[i + 1 + 6], force_param))
                                break;
                            if (!isnan(force_param) && !isinf(force_param))
                            {
                                leftarm.fctrl.paramK[i] = force_param;
                            }
                        }
                        for (i = 0;i < 6; i++)
                        {
                            if (!CM_Atof(cmd.param_list[i + 1 + 12], force_param))
                                break;
                            if (!isnan(force_param) && !isinf(force_param))
                            {
                                leftarm.fctrl.paramC[i] = 2 * force_param * sqrt( leftarm.fctrl.paramK[i] * leftarm.fctrl.paramM[i]);
                            }
                        }
                    }

                    printf("Rec to force\n");
                    leftarm.fctrl.Switch = -2;      // 第一周期需要清除力传感器，力控准备
                    memset(leftarm.fctrl.totalP, 0, sizeof(leftarm.fctrl.totalP));
                    memset(leftarm.fctrl.totalV, 0, sizeof(leftarm.fctrl.totalV));
                    memset(leftarm.fctrl.jointP, 0, sizeof(leftarm.fctrl.jointP));
                    memset(leftarm.fctrl.totalTrans, 0, sizeof(leftarm.fctrl.totalTrans));
                    leftarm.fctrl.totalTrans[0] = 1.0;
                    leftarm.fctrl.totalTrans[5] = 1.0;
                    leftarm.fctrl.totalTrans[10] = 1.0;
                    leftarm.fctrl.totalTrans[15] = 1.0;
                    leftarm.betaExp = FindBeta(leftarm.jointPos);

                    clearForceSensor(leftarm.endft);     // 清零力传感器
                }
                break;
            }

            case FORCE_DISABLE:
            {
                if (!CM_Atoi(cmd.param_list[0], left_right))
                    break;
                
                if (left_right == LEFT)
                {
                    if (leftarm.fctrl.Switch == 1)
                    {
                        printf("waiting for exit\n");
                        // leftarm.fctrl.Switch = -1;    // 慢速关闭
                        leftarm.fctrl.Switch = 2;       // 快速关闭
                    }
                }
                break;
            }
            
            case MOVE_TEST:
            {
                if (!CM_Atoi(cmd.param_list[0], left_right))
                    break;
                if (cmd.param_cnt == 1)
                {
                    test_A = 90.0 * DEG2RAD;      // 单位 度  最大运行范围（峰峰值）
                    test_T = 5.0;       // 单位 秒， 半周期（峰峰之间的时间）
                }
                else if (cmd.param_cnt == 3)
                {
                    if (!CM_Atof(cmd.param_list[1], test_A))
                        break;  
                    if (!CM_Atof(cmd.param_list[2], test_T))
                        break;  
                    test_A *= DEG2RAD;
                }

                if (left_right == LEFT)
                {
                    if (leftarm.state == IDLE)
                    {
                        leftarm.test_A = test_A;
                        leftarm.test_T = test_T;
                        leftarm.state = ON_MOVETEST;
                        leftarm.fctrl.Switch = 0;
                    }
                }
                else if (left_right == RIGHT)
                {
                    if (rightarm.state == IDLE)
                    {
                        rightarm.test_A = test_A;
                        rightarm.test_T = test_T;
                        rightarm.state = ON_MOVETEST;
                        rightarm.fctrl.Switch = 0;
                    }
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

            now = rt_timer_read();
            ecrt_master_application_time(master[0], (long long)now);
            ecrt_master_sync_reference_clock(master[0]);
            ecrt_master_sync_slave_clocks(master[0]);
            
            /********************** 遍历各身体部位进行控制 **********************/
            if (bodypart_use[LEFT]){
                ctrlArmMotor(leftarm);       // 控制左臂电机运动
            }

            robotSendFeedback(leftarm);

            for (i = 0; i< 7; i++){
                leftarm.motor[i].last_actposition = leftarm.motor[i].act_position;
            }

            break;

        case EXIT:

            break;
        }
        // 实时循环

        // queue process data
        for (i = 0; i< ETHERCAT_MAX; i++)
        {
            if (ethercat_use[i] == 1)
            {
                ecrt_domain_queue(domain[i].domain);
            }
        }

        // send process data
        for (i = 0; i< ETHERCAT_MAX; i++)
        {
            if (ethercat_use[i] == 1)
            {
                ecrt_master_send(master[i]);
            }
        }
      
        now = rt_timer_read();
        period = (now - previous) / 1000; //us
        leftarm.time_elapsed = double((now - start_time) / 1000000); //ms

        static time_t prev_second = 0;
        struct timeval tv;
        gettimeofday(&tv, 0);
        if (tv.tv_sec != prev_second)
        {
            // printf("program time elapsed: %.1fs, test time elapsed: %.1fs\n", leftarm.time_elapsed/1000.0, leftarm.test_time);
            ec_master_state_t a;
            rt_check_master_state(master[0], master_state[0]);
            
            prev_second = tv.tv_sec;
        }

        // printf("Timer peroid: %luus Loop time: %ldus\n", long(previous - last_moment)/1000, (long)period);
        // fprintf(fp, "Timer peroid: %luus Loop time: %ldus\n", long(previous - last_moment)/1000, (long)period);
        last_moment = previous;

    }

    regfree(&reg);

    // 状态改变可以生效，但是Switched ON 状态不上伺服
    rt_task_wait_period(NULL);

    uint32_t brake_output = 0x00000;
    if (bodypart_use[LEFT] == 1)
    {
        for (i = 0; i< leftarm.motornum; i++)
        {
            if (leftarm.motor_use[i] == 1){
                changeOneMotorState(leftarm, i, SWITCHED_ON);
                EC_WRITE_U32(domain[leftarm.dm_index].domain_pd + leftarm.motor[i].offset.DO, brake_output);  // 去使能成功加抱闸
            }
        }
    }
   
    // queue process data
    for (i = 0; i< ETHERCAT_MAX; i++)
    {
        if (ethercat_use[i] == 1)
        {
            ecrt_domain_queue(domain[i].domain);
        }
    }

    // send process data
    for (i = 0; i< ETHERCAT_MAX; i++)
    {
        if (ethercat_use[i] == 1)
        {
            ecrt_master_send(master[i]);
        }
    }
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
    int i;

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    // rt_print_auto_init(1);

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // ==================== 初始化插值数 ======================== //
    printf("Initializing interpolation param..\n");
    ip_param_init();

    // ==================== 配置UDP 通讯 ======================== //
    UDP_init();
    
    if ((fp = fopen("test.txt", "w")) == NULL);
	{
		printf("文件开始写入\n");
	}
	
    // ==================== 配置 EtherCAT 主站 ======================== //
    printf("Requesting master...\n");
    for (i = 0; i< ETHERCAT_MAX; i ++)
    {
        if (ethercat_use[i] == 1)
        {
            master[i] = ecrt_request_master(i);
            if (!master[i]) return -1;

            domain[i].domain = ecrt_master_create_domain(master[i]);
            if (!domain[i].domain) return -1;
        }
    }
    
    // ==================== 配置EtherCAT 从站 ======================== //
    printf("Creating slave configurations...\n");
    if (bodypart_use[LEFT] == 1){
        if (!leftarmInit(leftarm, LEFT_ETHERCAT, left_slave_pos)) 
            return -1;       // 配置左臂从站， 包括7个关节电机和1个力传感器, master0, domain0
        printf("leftarm init successed...\n");
    }
    
    // // ==================== 配置 EtherCAT PDO，激活主站，得到domain指针 ======================== //
    for (i = 0; i< ETHERCAT_MAX; i ++)
    {
        if (ethercat_use[i] == 1)
        {
            if (ArrayDomainRegs(domain[i]))
            {
                fprintf(stderr, "PDO domain entry registration failed!\n");
                return -1;
            }
        }
    }

    printf("Activating master...\n");
    for (i = 0; i< ETHERCAT_MAX; i ++)
    {
        if (ethercat_use[i] == 1)
        {
            if (ecrt_master_activate(master[i])) 
                return -1;
        
            if (!(domain[i].domain_pd = ecrt_domain_data(domain[i].domain)))
            {
                fprintf(stderr, "Failed to get domain data pointer.\n");
                return -1;
            }
        }
    }
    
    // // ============================== 实时进程 ============================== //
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
        // nsleep(2000000);
        sched_yield();
    }

    // ============================== 退出程序 ============================== //

    fclose(fp);

    printf("Deleting realtime task...\n");
    rt_task_delete(&my_task);

    printf("End of Program\n");
    for (i = 0; i< ETHERCAT_MAX; i ++)
    {
        if (ethercat_use[i] == 1)
        {    
            ecrt_release_master(master[i]);
        }
    }

    return 0;
}
