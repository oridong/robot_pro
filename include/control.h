# ifndef __CONTROL___

# define __CONTROL___

#include <errno.h>
#include <math.h>
#include <native/mutex.h>
#include <native/sem.h>
#include <native/task.h>
#include <native/timer.h>
#include <pthread.h>
#include <rtdk.h>
#include <rtdm/rtdm.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <queue>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "ecrt.h"

/* 电机Pdo地址偏移  */
typedef struct
{
    unsigned int target_position;
    unsigned int DO;
    unsigned int ctrl_word;
    unsigned int mode_operation;
    unsigned int act_position;
    unsigned int DI;
    unsigned int status_word;
    unsigned int ain;
} Offset;

/* 力传感器Pdo地址偏移  */
typedef struct
{
    unsigned int ctrl1;
    unsigned int ctrl2;
    unsigned int Fx;
    unsigned int Fy;
    unsigned int Fz;
    unsigned int Tx;
    unsigned int Ty;
    unsigned int Tz;
    unsigned int status1;
    unsigned int status2;
} Offset_ft;

/* EtherCAT domain结构体  */
typedef struct 
{
    ec_domain_t *domain;
    ec_domain_state_t domain_state;
    uint8_t *domain_pd;
    ec_pdo_entry_reg_t domain_regs[100];
    std::vector<ec_pdo_entry_reg_t> domain_reg;
}EC_domain;

/* EtherCAT 地址结构体 */
typedef struct EC_position
{
    uint16_t alias;
    uint16_t buspos;
} EC_position;

/* Elmo电机结构体 */
typedef struct 
{
    uint16_t alias;
    uint16_t buspos;
    Offset offset;
    uint16_t status;  /* DS402 status register, without manufacturer bits */
    ec_slave_config_t *sc_dig_out;

    uint8_t mode;
    uint8_t first_time;

    int act_position; /* actual position in cnt */
    int exp_position;   /* expect position in cnt */
    uint32_t ain;       // 模拟输入2 
    double this_send;

    std::vector<int> plan;
    double plan_param[4];
    uint16_t plan_cnt;
    double plan_run_time;
    uint16_t itp_period_times;

}Motor;

/* 力传感器 结构体 */
typedef struct 
{
    EC_position pos;
    Offset_ft offset;

    int32_t ft[6];
    bool dataReady;
    double countsPerForce;
    double countsPerTorque;

    uint32_t status;
    uint32_t sampCount;
}ft_sensor;

/* 插值 结构体 */
typedef struct 
{
    double para[17];
    double time;
}splan;

/* 身体部件 结构体 */
typedef struct 
{
    double T00[16];
    Motor motor[7];
    splan s_line;
    splan s_beta;
    splan s_equat;
    ft_sensor endft;
    int dm_index;
}bodypart;


int FT_sensor_init(bodypart arm, ec_master_t * m, int dm_index, EC_position pos);


# endif