
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

/******************* anOpen控制字 *******************/
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
#define ELMO_GOLD 0x0000009a, 0x00030924
#define ATI_FTSENSOR 0x00000732, 0x26483052

/***************************************************************
 * 变量及参数， 注意：电机参数列表需要大于使用电机数量
 /**************************************************************/
#define ctl_period 1000000 /* unit：ns， 1ms 控制周期 */
#define itp_window 3

#define MotorNum 1 // 使用的电机数量

const static EC_position Motor_Pos[] = {
    {0, 0},
    {0, 1},
    {0, 2},
};                                             // EtherCAT 电机总线地址
const static uint8_t Motor_Mode[] = {8, 8, 8}; // 与电机运行模式

// 插值
const uint8_t dip = 70;                       // 默认插值倍数（插值周期与控制周期比）
const uint8_t Motor_itp_times[] = {dip, dip}; // 在初始化时对电机设置，能够对不同电机进行不同的设置（可能在电机结构体初始化过程中更改更方便）

// 变量声明
Motor motor[10];
double alpha[itp_window] = {0.0f};
double alpha_sum = 0.0f;
RT_TASK my_task;

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

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};
static uint8_t *domain1_pd = NULL;
ec_pdo_entry_reg_t domain1_regs[8 * MotorNum + 1];

static ec_domain_t *domain2 = NULL;
static ec_domain_state_t domain2_state = {};
static uint8_t *domain2_pd = NULL;
ec_pdo_entry_reg_t domain2_regs[ 10 + 1];

/* Slave  */
ec_pdo_entry_info_t slave_pdo_entries[] = {
    {0x607A, 0x00, 32}, /* Target position */
    {0x60fe, 0x01, 32}, /* Physical outputs */
    {0x6040, 0x00, 16}, /* Controlword */
    {0x6060, 0x00, 8},  /* mode of operation */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x60fd, 0x00, 32}, /* Digital inputs */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x2203, 0x00, 32}  /* Analog Input*/
};

ec_pdo_info_t slave_pdos[] = {
    {0x1600, 3, slave_pdo_entries + 0}, /* RPDO1 Mapping */
    {0x160B, 1, slave_pdo_entries + 3}, /* RPDO1 Mapping */
    {0x1a00, 3, slave_pdo_entries + 4}, /* TPDO1 Mapping */
    {0x1A24, 1, slave_pdo_entries + 7}  /* TPDO1 Mapping */
};

ec_sync_info_t slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, slave_pdos + 2, EC_WD_DISABLE},
    {0xff}
};

Offset_ft offset_ft;
/*****************************************************************************
 * Ethercat 检查
 ****************************************************************************/

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

/****************************************************************************/

void rt_check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master1, &ms);

    if (ms.slaves_responding != master1_state.slaves_responding)
    {
        rt_printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master1_state.al_states)
    {
        rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master1_state.link_up)
    {
        rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master1_state = ms;
}

void nsleep(long nano)
{
    struct timespec sleeptime;
    sleeptime.tv_nsec = nano;
    sleeptime.tv_sec = 0;
    nanosleep(&sleeptime, NULL);
}

/*****************************************************************************
 * Motor 结构体初始化
 ****************************************************************************/

void motorStructInit()
{
    int i = 0;
    for (i = 0; i < MotorNum; i++)
    {
        motor[i].alias = Motor_Pos[i].alias;
        motor[i].buspos = Motor_Pos[i].buspos;

        ec_pdo_entry_reg_t temp1 = {Motor_Pos[i].alias, Motor_Pos[i].buspos, ELMO_GOLD, 0x607A, 0, &motor[i].offset.target_position, NULL};
        ec_pdo_entry_reg_t temp2 = {Motor_Pos[i].alias, Motor_Pos[i].buspos, ELMO_GOLD, 0x60FE, 1, &motor[i].offset.DO, NULL};
        ec_pdo_entry_reg_t temp3 = {Motor_Pos[i].alias, Motor_Pos[i].buspos, ELMO_GOLD, 0x6040, 0, &motor[i].offset.ctrl_word, NULL};
        ec_pdo_entry_reg_t temp4 = {Motor_Pos[i].alias, Motor_Pos[i].buspos, ELMO_GOLD, 0x6060, 0, &motor[i].offset.mode_operation, NULL};
        ec_pdo_entry_reg_t temp5 = {Motor_Pos[i].alias, Motor_Pos[i].buspos, ELMO_GOLD, 0x6064, 0, &motor[i].offset.act_position, NULL};
        ec_pdo_entry_reg_t temp6 = {Motor_Pos[i].alias, Motor_Pos[i].buspos, ELMO_GOLD, 0x60fd, 0, &motor[i].offset.DI, NULL};
        ec_pdo_entry_reg_t temp7 = {Motor_Pos[i].alias, Motor_Pos[i].buspos, ELMO_GOLD, 0x6041, 0, &motor[i].offset.status_word, NULL};
        ec_pdo_entry_reg_t temp8 = {Motor_Pos[i].alias, Motor_Pos[i].buspos, ELMO_GOLD, 0x2203, 0, &motor[i].offset.ain, NULL};
        memcpy(&domain1_regs[i * 8 + 0], &temp1, sizeof(temp1));
        memcpy(&domain1_regs[i * 8 + 1], &temp2, sizeof(temp2));
        memcpy(&domain1_regs[i * 8 + 2], &temp3, sizeof(temp3));
        memcpy(&domain1_regs[i * 8 + 3], &temp4, sizeof(temp4));
        memcpy(&domain1_regs[i * 8 + 4], &temp5, sizeof(temp5));
        memcpy(&domain1_regs[i * 8 + 5], &temp6, sizeof(temp6));
        memcpy(&domain1_regs[i * 8 + 6], &temp7, sizeof(temp7));
        memcpy(&domain1_regs[i * 8 + 7], &temp8, sizeof(temp8));

        motor[i].first_time = 0;
        motor[i].exp_position = 0;
        motor[i].act_position = 0;
        motor[i].mode = Motor_Mode[i];
        motor[i].this_send = 0;
        motor[i].itp_period_times = Motor_itp_times[i];
        motor[i].plan_cnt = 0;
        motor[i].plan_run_time = 0.0f;
    }
    ec_pdo_entry_reg_t temp9 = {};
    memcpy(&domain1_regs[MotorNum * 8], &temp9, sizeof(domain1_regs[MotorNum * 8]));

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


int FT_sensor_init()
{
    ec_pdo_entry_info_t slave_0_pdo_entries[] = {
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

    ec_pdo_info_t slave_0_pdos[] = {
        {0x1601, 2, slave_0_pdo_entries + 0}, /* DO RxPDO-Map */
        {0x1a00, 8, slave_0_pdo_entries + 2}, /* DI TxPDO-Map */
    };

    ec_sync_info_t slave_0_syncs[] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
        {0xff}
    };

    ec_slave_config_t *ft_sc_dig_out;
    
    ft_sc_dig_out = ecrt_master_slave_config(master1, 0, 0, ATI_FTSENSOR);
    if (!ft_sc_dig_out)
    {
        fprintf(stderr, "Failed to get slave configuration, \n");
        return -1;
    }
    if (ecrt_slave_config_pdos(ft_sc_dig_out, EC_END, slave_0_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    ec_pdo_entry_reg_t temp1 = {0,0, ATI_FTSENSOR, 0x7010, 1, &offset_ft.ctrl1, NULL};
    ec_pdo_entry_reg_t temp2 = {0,0, ATI_FTSENSOR, 0x7010, 2, &offset_ft.ctrl2, NULL};
    ec_pdo_entry_reg_t temp3 = {0,0, ATI_FTSENSOR, 0x6000, 1, &offset_ft.Fx, NULL};
    ec_pdo_entry_reg_t temp4 = {0,0, ATI_FTSENSOR, 0x6000, 2, &offset_ft.Fy, NULL};
    ec_pdo_entry_reg_t temp5 = {0,0, ATI_FTSENSOR, 0x6000, 3, &offset_ft.Fz, NULL};
    ec_pdo_entry_reg_t temp6 = {0,0, ATI_FTSENSOR, 0x6000, 4, &offset_ft.Tx, NULL};
    ec_pdo_entry_reg_t temp7 = {0,0, ATI_FTSENSOR, 0x6000, 5, &offset_ft.Ty, NULL};
    ec_pdo_entry_reg_t temp8 = {0,0, ATI_FTSENSOR, 0x6000, 6, &offset_ft.Tz, NULL};
    ec_pdo_entry_reg_t temp9 = {0,0, ATI_FTSENSOR, 0x6010, 0, &offset_ft.status1, NULL};
    ec_pdo_entry_reg_t temp10 = {0,0, ATI_FTSENSOR, 0x6020, 0, &offset_ft.status2, NULL};
    memcpy(&domain2_regs[0], &temp1, sizeof(temp1));
    memcpy(&domain2_regs[1], &temp2, sizeof(temp2));
    memcpy(&domain2_regs[2], &temp3, sizeof(temp3));
    memcpy(&domain2_regs[3], &temp4, sizeof(temp4));
    memcpy(&domain2_regs[4], &temp5, sizeof(temp5));
    memcpy(&domain2_regs[5], &temp6, sizeof(temp6));
    memcpy(&domain2_regs[6], &temp7, sizeof(temp7));
    memcpy(&domain2_regs[7], &temp8, sizeof(temp8));
    memcpy(&domain2_regs[8], &temp9, sizeof(temp9));
    memcpy(&domain2_regs[9], &temp10, sizeof(temp10));

    ec_pdo_entry_reg_t temp11 = {};
    memcpy(&domain2_regs[10], &temp11, sizeof(temp11));

    uint32_t data1 = 0x000;
    uint32_t abort_code;
    if (!ecrt_master_sdo_download(master1, 0, 0x7010, 1, (uint8_t *)&data1, sizeof(data1), &abort_code)) // 写SDO，0x2F41:0 16-19位配置为4 即可开启模拟二通道输入
    {
        printf("Set successed!\n");
    }

    uint8_t result[4];
    size_t target_size = 4;
    size_t result_size;

    if (ecrt_master_sdo_upload(master1, 0, 0x2040, 0x31, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
    {
        fprintf(stderr, "Failed to get sdo data.\n");
        return -1;
    }
    uint32_t * data = (uint32_t *)result;
    printf("%f,%d\n",(double(*data)), result_size);

    target_size = 4;
    result_size;

    if (ecrt_master_sdo_upload(master1, 0, 0x6000, 0x03, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
    {
        fprintf(stderr, "Failed to get sdo data.\n");
        return -1;
    }
    int32_t data2 = 0;
    data2 = result[3] << 24 | result[2] << 16 | result[1] << 8 | result[0];
    printf("%f\n",(float)data2/1000000.0);

    
}

int interpolation(Motor *m)
{
    double p1 = 0;
    double p2 = 0;
    double v1 = 0;
    double v2 = 0;
    double tf = double(m->itp_period_times) * double(ctl_period) / 1e9;

    while (m->plan.size() <= itp_window + 1)
    {
        m->plan.push_back(m->exp_position);
    }

    if (m->plan.size() > itp_window + 1) //保持itp_window+1长度的队列
    {
        m->plan.erase(m->plan.begin());

        for (int j = 0; j < itp_window; j++) //遍历列表取数
        {
            p1 += m->plan[j] * alpha[j] / alpha_sum;
            p2 += m->plan[j + 1] * alpha[j] / alpha_sum;
        }
        v1 = (m->plan[itp_window - 1] - m->plan[0]) / ((itp_window - 1) * tf);
        v2 = (m->plan[itp_window] - m->plan[1]) / ((itp_window - 1) * tf);

        m->plan_param[0] = p1;
        m->plan_param[1] = v1;
        m->plan_param[2] = 3.0f / (tf * tf) * (p2 - p1) - 2.0f / tf * v1 - 1.0f / tf * v2;
        m->plan_param[3] = -2.0f / (tf * tf * tf) * (p2 - p1) + 1.0f / (tf * tf) * (v2 + v1);
    }
}

/*****************************************************************************
 * EtherCat写帧-改变电机状态-状态机
 ****************************************************************************/

uint8_t changeOneMotorState(uint8_t id, uint8_t state)
{
    /* read inputs */
    motor[id].status = EC_READ_U16(domain1_pd + motor[id].offset.status_word);

    //DS402 CANOpen over EtherCAT status machine
    if ((motor[id].status & 0x004f) == 0x0008) // Fault
    {
        EC_WRITE_U16(domain1_pd + motor[id].offset.ctrl_word, FaultReset);
        return 0;
    }

    if ((motor[id].status & 0x004f) == 0x0040) // Switch on disable
    {
        if (state == SWITCH_ON_DISABLED)
        {
            return 1;
        }
        else
        {
            EC_WRITE_U16(domain1_pd + motor[id].offset.ctrl_word, SM_trans2);
        }
    }
    else if ((motor[id].status & 0x006f) == 0x0021) //Ready to switch on
    {
        if (state == READY_TO_SWITCH_ON)
        {
            return 1;
        }
        else
        {
            EC_WRITE_U16(domain1_pd + motor[id].offset.ctrl_word, SM_trans3);
        }
    }
    else if ((motor[id].status & 0x006f) == 0x0023) //switch on
    {
        if (state == SWITCHED_ON)
        {
            return 1;
        }
        else
        {
            EC_WRITE_U16(domain1_pd + motor[id].offset.ctrl_word, SM_trans4);
        }
    }
    else if ((motor[id].status & 0x006f) == 0x0027) //operation enabled
    {
        if (state == OPERATION_ENABLE)
        {
            return 1;
        }
        else if (state == SWITCHED_ON)
        {
            EC_WRITE_U16(domain1_pd + motor[id].offset.ctrl_word, SM_trans5);
        }
        else if (state == QUICK_STOP_ACTIVE || state == SWITCH_ON_DISABLED)
        {
            EC_WRITE_U16(domain1_pd + motor[id].offset.ctrl_word, QuickStop);
        }
    }
    return 0;
}

/*****************************************************************************
 * EtherCat写帧-改变多个电机状态
 ****************************************************************************/

uint8_t changeMotorState(int8_t id, uint8_t state)
{
    int i = 0;
    uint8_t ret = 1;
    uint8_t ret_t;
    if (id == -1)
    {
        for (i = 0; i < MotorNum; i++)
        {
            ret_t = changeOneMotorState(i, state);
            ret &= ret_t;
        }
    }
    else
    {
        ret = changeOneMotorState(id, state);
    }
    return ret;
}

/*****************************************************************************
 * 实时任务函数
 ****************************************************************************/

int32_t f[6];
void my_task_proc(void *arg)
{
    // 系统时间
    RTIME now, previous, period;
    rt_task_set_periodic(NULL, TM_NOW, RTIME(ctl_period)); // unit :ns
    uint8_t run_state = CONFIG_ELMO;

    int i;
    uint8_t ready = 0;
    float t = 0;
    command cmd;
    uint32_t status1;
    uint32_t sampcounter;

    while (run)
    {
        rt_task_wait_period(NULL);      // 等待定时周期
        previous = rt_timer_read();     // 获取运行周期开始时间

        // 接收 EtherCAT 帧
        ecrt_master_receive(master0);
        ecrt_master_receive(master1);

        ecrt_domain_process(domain1);
        ecrt_domain_process(domain2);

        f[0] = EC_READ_S32(domain2_pd + offset_ft.Fx);
        f[1] = EC_READ_S32(domain2_pd + offset_ft.Fy);
        f[2] = EC_READ_S32(domain2_pd + offset_ft.Fz);
        f[3] = EC_READ_S32(domain2_pd + offset_ft.Tx);
        f[4] = EC_READ_S32(domain2_pd + offset_ft.Ty);
        f[5] = EC_READ_S32(domain2_pd + offset_ft.Tz);

        status1 = EC_READ_U32(domain2_pd + offset_ft.status1);
        sampcounter = EC_READ_U32(domain2_pd + offset_ft.status2);

        printf("%d,%d,%d,%d,%d,%d\n",f[0], f[1], f[2], f[3], f[4], f[5]);

        switch (run_state)
        {

        case CONFIG_ELMO:

            for (i = 0; i < MotorNum; i++)
            {
                EC_WRITE_U8(domain1_pd + motor[i].offset.mode_operation, motor[i].mode);
            }
            ready = changeMotorState(-1, SWITCHED_ON);

            // printf("time: %ld, ready:%d\n", (long)previous, ready);
            if (ready)
            {
                run_state = ENABLE;
                printf("Elmo Configuration Finished.\n");
            }
            break;

        case ENABLE:

            ready = changeMotorState(-1, OPERATION_ENABLE);
            if (ready)
            {
                printf("Elmo Enable Finished.\n");
                run_state = CONTROL;
            }
            break;

        case CONTROL:

            changeMotorState(-1, OPERATION_ENABLE); // 清除电机错误

            /********************** 读取当前位置 更新结构体 **********************/
            for (i = 0; i < MotorNum; i++)
            {
                motor[i].act_position = EC_READ_S32(domain1_pd + motor[i].offset.act_position);
                motor[i].ain = EC_READ_U32(domain1_pd + motor[i].offset.ain);
                if (motor[i].first_time == 0) // 初次进入，记录开机时刻位置作为期望位置
                {
                    printf("first:%d, act_position:%d\n", i, motor[i].act_position);
                    motor[i].exp_position = motor[i].act_position;
                    motor[i].first_time = 1;
                }
            }

            /********************** 接受并解析指令 根据指令进行控制 **********************/
            cmd = robotReceiveCommand();

            switch (cmd.cmd_mode)
            {
            case STOP_ALL:
                for (i = 0; i < MotorNum; i++)
                {
                    motor[i].exp_position = motor[i].act_position;
                    motor[i].plan_cnt = 0;
                    motor[i].plan.clear();
                }
                break;
            case CAR_MOVE:

                break;

            case HEAD_MOVE:
                motor[0].exp_position = atoi(cmd.param_list[0]);
                motor[1].exp_position = atoi(cmd.param_list[0]);
                motor[2].exp_position = atoi(cmd.param_list[2]);
                printf("%d,%d,%d\n", motor[0].exp_position, motor[1].exp_position, motor[2].exp_position);
                break;
            case NO_RECV:

                break;
            case ERROR_MATCH:

                break;
            }

            /********************** 遍历各电机进行控制 **********************/
            for (i = 0; i < MotorNum; i++)
            {
                /********************** 电机轨迹插值运动 **********************/

                if (motor[i].plan_cnt == 0) //规划周期到达，进行插值规划
                {
                    interpolation(&motor[i]);
                }
                motor[i].plan_run_time = double(motor[i].plan_cnt) * double(ctl_period) / 1e9;
                motor[i].this_send = motor[i].plan_param[0] + motor[i].plan_param[1] * motor[i].plan_run_time + motor[i].plan_param[2] * motor[i].plan_run_time * motor[i].plan_run_time + motor[i].plan_param[3] * motor[i].plan_run_time * motor[i].plan_run_time * motor[i].plan_run_time;

                motor[i].plan_cnt++;
                if (motor[i].plan_cnt == motor[i].itp_period_times)
                {
                    motor[i].plan_cnt = 0;
                }

                /********************** 填写指令，等待发送 **********************/
                EC_WRITE_S32(domain1_pd + motor[i].offset.target_position, int(motor[i].this_send));
            }

            robotSendFeedback(motor);

            break;

        case EXIT:

            break;
        }

        static time_t prev_second = 0;
        struct timeval tv;
        gettimeofday(&tv, 0);
        if (tv.tv_sec != prev_second)
        {
            // printf( "AKD: Motor = 0; act_position = %d, exp_position = %d, status = 0x%x, Loop time : %ldus\n",
            //     motor[0].act_position,  motor[0].exp_position, motor[0].status, (long)period);
            prev_second = tv.tv_sec;
        }

        // send process data
        ecrt_domain_queue(domain1);
        ecrt_domain_queue(domain2);
        ecrt_master_send(master1);
        ecrt_master_send(master0);
        now = rt_timer_read();
        period = (now - previous) / 1000; //us
    }
    regfree(&reg);

    // 状态改变可以生效，但是Switched ON 状态不上伺服
    rt_task_wait_period(NULL);
    // changeMotorState(-1, SWITCHED_ON);

    ecrt_domain_queue(domain1);
    ecrt_domain_queue(domain2);
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

    // ==================== 初始化电机结构体 ======================== //
    printf("Initializing motor structure...\n");
    motorStructInit();

    // ==================== 配置UDP 通讯 ======================== //
    UDP_init();

    // ==================== 配置 EtherCAT 主站 ======================== //
    printf("Requesting master...\n");
    master0 = ecrt_request_master(0);
    if (!master0)
        return -1;
    master1 = ecrt_request_master(1);
    if (!master1)
        return -1;

    domain1 = ecrt_master_create_domain(master0);
    if (!domain1)
        return -1;
    domain2 = ecrt_master_create_domain(master1);
    if (!domain2)
        return -1;

    FT_sensor_init();
    // ==================== 配置EtherCAT 从站 ======================== //
    printf("Creating slave configurations...\n");
    int i = 0;
    for (i = 0; i < MotorNum; i++)
    {
        motor[i].sc_dig_out = ecrt_master_slave_config(master0, motor[i].alias, motor[i].buspos, ELMO_GOLD);
        if (!motor[i].sc_dig_out)
        {
            fprintf(stderr, "Failed to get slave configuration, No.%d.\n", i);
            return -1;
        }
        if (ecrt_slave_config_pdos(motor[i].sc_dig_out, EC_END, slave_syncs))
        {
            fprintf(stderr, "Failed to configure PDOs. No.%d.\n", i);
            return -1;
        }

        // ==================== 读写 SDO，配置AD-input2到PDO中 ======================== //
        uint8_t result[4];
        size_t target_size = 4;
        size_t result_size;
        uint32_t abort_code;

        if (ecrt_master_sdo_upload(master0, motor[i].buspos, 0x2F41, 0, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
        {
            fprintf(stderr, "Failed to get sdo data.\n");
            return -1;
        }

        uint32_t data = 0x40000;
        uint8_t *data_send;
        data_send = (uint8_t *)&data;
        size_t data_size = sizeof(data);

        if (!ecrt_master_sdo_download(master0, motor[i].buspos, 0x2f41, 0, data_send, data_size, &abort_code)) // 写SDO，0x2F41:0 16-19位配置为4 即可开启模拟二通道输入
        {
            printf("Set App to analog input 2 successed!\n");
        }

        if (ecrt_master_sdo_upload(master0, motor[i].buspos, 0x2F41, 0, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
        {
            fprintf(stderr, "Failed to get sdo data.\n");
            return -1;
        }
        // =========================================================================== //
    }

    // ==================== 配置 EtherCAT PDO，激活主站，得到domain指针 ======================== //
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs))
    {
        fprintf(stderr, "PDO domain1 entry registration failed!\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain2, domain2_regs))
    {
        fprintf(stderr, "PDO domain2 entry registration failed!\n");
        return -1;
    }
  
    printf("Activating master...\n");
    if (ecrt_master_activate(master1)) return -1;
    if (ecrt_master_activate(master0)) return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1)))
    {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        return -1;
    }
    if (!(domain2_pd = ecrt_domain_data(domain2)))
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
    ret = rt_task_start(&my_task, &my_task_proc, NULL);
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
