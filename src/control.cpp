
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

// EtherCAT 电机总线地址
static EC_position left_slave_pos[] = {{0, 0}, {0, 1}, {0, 2}, {0, 2}, {0, 2}, {0, 2}, {0, 2}, {0, 0},};// 力传感器位置,   

const static uint8_t armMotorMode = 8; // 机械臂电机运行模式

// 插值周期
const uint8_t armMotoritpTimes[] = {70, 70, 70, 70, 70, 70, 70}; // 在初始化时对电机设置，能够对不同电机进行不同的设置, 默认插值倍数（插值周期与控制周期比）为70

// 变量声明
bodypart leftarm;
bodypart rightarm;
bodypart head;
bodypart track;

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

EC_domain domain[5];

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};
static uint8_t *domain1_pd = NULL;
ec_pdo_entry_reg_t domain1_regs[8 * MotorNum + 1];
std::vector<ec_pdo_entry_reg_t> doma1_reg;

/* Slaves */ 
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

/*****************************************************************************
 * 初始化插值位置滤波器参数
 ****************************************************************************/
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

/*****************************************************************************
/* 功能： 将左臂初始化为位置模式 
*  输入：记录结构体
*       主站序号
*       总线位置数组
*****************************************************************************/
int leftarmInit(bodypart &arm, ec_master_t *m, int dm_index, EC_position * motor_pos)
{
    arm.dm_index = dm_index;        // 记录当前身体部分所使用的domain
    int i = 0;
    for (i = 0; i < sizeof(arm.motor)/sizeof(Motor); i++)
    {
        arm.motor[i].alias = motor_pos->alias;
        arm.motor[i].buspos = motor_pos->buspos;

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

        arm.motor[i].first_time = 0;
        arm.motor[i].exp_position = 0;
        arm.motor[i].act_position = 0;
        arm.motor[i].mode = armMotorMode;
        arm.motor[i].this_send = 0;
        arm.motor[i].itp_period_times = armMotoritpTimes[i];
        arm.motor[i].plan_cnt = 0;
        arm.motor[i].plan_run_time = 0.0f;

        arm.motor[i].sc_dig_out = ecrt_master_slave_config(m, motor_pos[i].alias, motor_pos[i].buspos, ELMO_GOLD);
        if (!arm.motor[i].sc_dig_out)
        {
            fprintf(stderr, "Failed to get slave configuration, No.%d.\n", i);
            return -1;
        }
        if (ecrt_slave_config_pdos(arm.motor[i].sc_dig_out, EC_END, ss_pos))
        {
            fprintf(stderr, "Failed to configure PDOs. No.%d.\n", i);
            return -1;
        }

        // ==================== 读写 SDO，配置0x2f41:0x00, 加入AD-input2到PDO中 ======================== //
        // 1、读入0x2f41:0x00 当前的值
        uint8_t result[4];
        size_t target_size = 4;
        size_t result_size;
        uint32_t abort_code;

        if (ecrt_master_sdo_upload(master0, arm.motor[i].buspos, 0x2F41, 0, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
        {
            fprintf(stderr, "Failed to get sdo data.\n");
            return -1;
        }

        // 2、写入0x40000 到0x2f41:0x00 配置用户自定义 pdo 的功能
        uint32_t data = 0x40000;
        uint8_t *data_send = (uint8_t *)&data;
        size_t data_size = sizeof(data);
        if (!ecrt_master_sdo_download(master0, arm.motor[i].buspos, 0x2f41, 0, data_send, data_size, &abort_code)) // 写SDO，0x2F41:0 16-19位配置为4 即可开启模拟二通道输入
        {
            printf("Set App to analog input 2 successed!\n");
        }

        // 3、重读检验是否成功写入
        if (ecrt_master_sdo_upload(master0, arm.motor[i].buspos, 0x2F41, 0, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
        {
            fprintf(stderr, "Failed to get sdo data.\n");
            return -1;
        }
        // =========================================================================== //

    } /* motor 循环*/

    // 初始化力传感器从站
    EC_position ft_pos = {motor_pos[7].alias, motor_pos[7].buspos};
    i = FT_sensor_init(arm, m, dm_index, ft_pos);
    return i;
}

int FT_sensor_init(bodypart arm, ec_master_t * m, int dm_index, EC_position pos)
{
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
        return -1;
    }
    if (ecrt_slave_config_pdos(ft_sc_dig_out, EC_END, ss_ft))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    ec_pdo_entry_reg_t temp1 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[0].index, spe_ft[0].subindex, &arm.endft.offset.ctrl1, NULL};
    ec_pdo_entry_reg_t temp2 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[0].index, spe_ft[0].subindex, &arm.endft.offset.ctrl2, NULL};
    ec_pdo_entry_reg_t temp3 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[0].index, spe_ft[0].subindex, &arm.endft.offset.Fx, NULL};
    ec_pdo_entry_reg_t temp4 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[0].index, spe_ft[0].subindex, &arm.endft.offset.Fy, NULL};
    ec_pdo_entry_reg_t temp5 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[0].index, spe_ft[0].subindex, &arm.endft.offset.Fz, NULL};
    ec_pdo_entry_reg_t temp6 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[0].index, spe_ft[0].subindex, &arm.endft.offset.Tx, NULL};
    ec_pdo_entry_reg_t temp7 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[0].index, spe_ft[0].subindex, &arm.endft.offset.Ty, NULL};
    ec_pdo_entry_reg_t temp8 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[0].index, spe_ft[0].subindex, &arm.endft.offset.Tz, NULL};
    ec_pdo_entry_reg_t temp9 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[0].index, spe_ft[0].subindex, &arm.endft.offset.status1, NULL};
    ec_pdo_entry_reg_t temp10 = {pos.alias, pos.buspos, ATI_FTSENSOR, spe_ft[0].index, spe_ft[0].subindex, &arm.endft.offset.status2, NULL};

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
    uint32_t data1 = 0x000;
    uint32_t abort_code;
    if (ecrt_master_sdo_download(m, pos.buspos, 0x7010, 0x01, (uint8_t *)&data1, sizeof(data1), &abort_code)) // 写SDO，0x2F41:0 16-19位配置为4 即可开启模拟二通道输入
    {
        fprintf(stderr, "Failed to set Control Word\n");
    }

    uint8_t result[4];
    size_t target_size = 4;
    size_t result_size;
    uint32_t *data;

    // 2、从0x2040:0x31 读Sdo 得到Counts per Force
    if (ecrt_master_sdo_upload(m, pos.buspos, 0x2040, 0x31, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
    {
        fprintf(stderr, "Failed to get sdo data.\n");
        return -1;
    }
    data = (uint32_t *)result;
    arm.endft.countsPerForce = double(*data);      // 将读到的数据写入手臂结构体
    // printf("%f,%d\n",(double(*data)), result_size);

    // 3、从0x2040:0x32 读Sdo 得到Counts per Torque
    if (ecrt_master_sdo_upload(m, pos.buspos, 0x2040, 0x32, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
    {
        fprintf(stderr, "Failed to get sdo data.\n");
        return -1;
    }
    data = (uint32_t *)result;
    arm.endft.countsPerTorque = double(*data);      // 将读到的数据写入手臂结构体
    // printf("%f,%d\n",(double(*data)), result_size);

    // 4、从0x6000:0x03 读Sdo 试读力传感器数据Fz
    // if (ecrt_master_sdo_upload(m, pos.buspos, 0x6000, 0x03, result, target_size, &result_size, &abort_code)) // 读SDO， 0x2F41:0 为用户应用配置字
    // {
    //     fprintf(stderr, "Failed to get sdo data.\n");
    //     return -1;
    // }
    // data1 = result[3] << 24 | result[2] << 16 | result[1] << 8 | result[0];
    // printf("%f\n",(float)data1/1000000.0);

}

int ArrayDomainRegs(EC_domain &dm)
{
    int i = 0;
    for (i = 0; i < dm.domain_reg.size(); i++)
    {
        memcpy(&dm.domain_regs[i], &dm.domain_reg.at(i), sizeof(dm.domain_reg.at(i)));
    }
    ec_pdo_entry_reg_t temp = {};
    memcpy(&dm.domain_regs[i], &temp, sizeof(temp));

    i = ecrt_domain_reg_pdo_entry_list(dm.domain, dm.domain_regs);
    return i;
}

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

/*****************************************************************************
 * EtherCat写帧-改变电机状态-状态机
 ****************************************************************************/
uint8_t changeOneMotorState(bodypart &arm, int8_t id, uint8_t state)
{
    Motor *m = &arm.motor[id];
    int dm_index = arm.dm_index;
    /* read inputs */
    m->status = EC_READ_U16(domain[dm_index].domain_pd + m->offset.status_word);

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

/*****************************************************************************
 * EtherCat写帧-改变多个电机状态
 ****************************************************************************/
uint8_t changeBodyMotorState(bodypart &arm, int8_t id, uint8_t state)
{
    int i = 0;
    uint8_t ret = 1;
    uint8_t ret_t;
    if (id == -1)
    {
        for (i = 0; i < sizeof(arm.motor)/sizeof(Motor); i++)
        {
            ret_t = changeOneMotorState(arm, id, state);
            ret &= ret_t;
        }
    }
    else
    {
        ret = changeOneMotorState(arm, id, state);
    }
    return ret;
}

void stopArmMotor(bodypart & arm)
{
    int i;
    int motornum = sizeof(arm.motor)/sizeof(Motor);
    for (i = 0; i < motornum; i++)
    {
        arm.motor[i].exp_position = arm.motor[i].act_position;
        arm.motor[i].plan_cnt = 0;
        arm.motor[i].plan.clear();
    }
}

void readArmData(bodypart & arm)
{
    int i;
    int motornum = sizeof(arm.motor)/sizeof(Motor);
    for (i = 0; i < motornum; i++)
    {
        arm.motor[i].act_position = EC_READ_S32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.act_position);
        arm.motor[i].ain = EC_READ_U32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.ain);
        if (arm.motor[i].first_time == 0) // 初次进入，记录开机时刻位置作为期望位置
        {
            printf("first:%d, act_position:%d\n", i, arm.motor[i].act_position);
            arm.motor[i].exp_position = arm.motor[i].act_position;
            arm.motor[i].first_time = 1;
        }
    }
}

void readForceData(bodypart &arm)
{
    int dm_index = arm.dm_index;
    arm.endft.ft[0] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Fx)/arm.endft.countsPerForce;
    arm.endft.ft[1] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Fy)/arm.endft.countsPerForce;
    arm.endft.ft[2] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Fz)/arm.endft.countsPerForce;
    arm.endft.ft[3] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Tx)/arm.endft.countsPerTorque;
    arm.endft.ft[4] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Ty)/arm.endft.countsPerTorque;
    arm.endft.ft[5] = (double)EC_READ_S32(domain[dm_index].domain_pd + arm.endft.offset.Tz)/arm.endft.countsPerTorque;

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
    else
    {
        arm.endft.dataReady = 0;
    }

    printf("%d,%d,%d,%d,%d,%d\n",arm.endft.ft[0], arm.endft.ft[1], arm.endft.ft[2], arm.endft.ft[3], arm.endft.ft[4], arm.endft.ft[5]);

}

void ctrlArm(bodypart &arm)
{
    int i;
    int motornum = sizeof(arm.motor)/sizeof(Motor);
    for (i = 0; i < motornum; i++)
    {
        /********************** 电机轨迹插值运动 **********************/
        if (arm.motor[i].plan_cnt == 0) //规划周期到达，进行插值规划
        {
            interpolation(arm.motor[i]);
        }
        arm.motor[i].plan_run_time = double(arm.motor[i].plan_cnt) * double(ctl_period) / 1e9;
        arm.motor[i].this_send = arm.motor[i].plan_param[0] + arm.motor[i].plan_param[1] * arm.motor[i].plan_run_time + arm.motor[i].plan_param[2] * arm.motor[i].plan_run_time * arm.motor[i].plan_run_time + arm.motor[i].plan_param[3] * arm.motor[i].plan_run_time * arm.motor[i].plan_run_time * arm.motor[i].plan_run_time;

        arm.motor[i].plan_cnt++;
        if (arm.motor[i].plan_cnt == arm.motor[i].itp_period_times)
        {
            arm.motor[i].plan_cnt = 0;
        }

        /********************** 填写指令，等待发送 **********************/
        EC_WRITE_S32(domain[arm.dm_index].domain_pd + arm.motor[i].offset.target_position, int(arm.motor[i].this_send));
    }

}
/*****************************************************************************
 * 实时任务函数
 ****************************************************************************/

void realtime_proc(void *arg)
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

        ecrt_domain_process(domain[0].domain);
        ecrt_domain_process(domain[1].domain);

        readForceData(leftarm);

        switch (run_state)
        {

        case CONFIG_ELMO:

            for (i = 0; i < sizeof(leftarm.motor)/sizeof(Motor); i++)
            {
                EC_WRITE_U8(domain[leftarm.dm_index].domain_pd + leftarm.motor[i].offset.mode_operation, leftarm.motor[i].mode);
            }
            ready = changeBodyMotorState(leftarm, -1, SWITCHED_ON);

            // printf("time: %ld, ready:%d\n", (long)previous, ready);
            if (ready)
            {
                run_state = CONTROL;
                printf("Elmo Mode Configuration Finished.\n");
            }
            break;

        case ENABLE:

            ready = changeBodyMotorState(leftarm, -1, OPERATION_ENABLE);
            if (ready)
            {
                printf("Elmo Enable Finished.\n");
                run_state = CONTROL;
            }
            break;

        case CONTROL:

            /********************** 读取当前位置 更新结构体 **********************/
            readArmData(leftarm);
            

            /********************** 接受并解析指令 根据指令进行控制 **********************/
            cmd = robotReceiveCommand();

            switch (cmd.cmd_mode)
            {
            case STOP_ALL:
                stopArmMotor(leftarm);
                
                break;

            case ENABLE_MOTOR:
                break;

            case CAR_MOVE:

                break;

            case HEAD_MOVE:
                head.motor[0].exp_position = atoi(cmd.param_list[0]);
                head.motor[1].exp_position = atoi(cmd.param_list[0]);
                head.motor[2].exp_position = atoi(cmd.param_list[2]);
                printf("%d,%d,%d\n", head.motor[0].exp_position, head.motor[1].exp_position, head.motor[2].exp_position);
                break;
            case NO_RECV:

                break;
            case ERROR_MATCH:

                break;
            }

            /********************** 遍历各电机进行控制 **********************/
            ctrlArm(leftarm);       // 控制左臂电机运动
            
            robotSendFeedback(leftarm, rightarm, head, track);

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
        ecrt_domain_queue(domain[0].domain);
        ecrt_domain_queue(domain[1].domain);
        ecrt_master_send(master1);
        ecrt_master_send(master0);
        now = rt_timer_read();
        period = (now - previous) / 1000; //us
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
    leftarmInit(leftarm, master0, 0, left_slave_pos);       // 配置左臂从站， 包括7个关节电机和1个力传感器, master0, domain0



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
    if (ecrt_master_activate(master0)) return -1;
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
