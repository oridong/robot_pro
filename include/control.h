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
#include <stdbool.h>
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

#define ctl_period 1000000 /* unit：ns， 1ms 控制周期 */
#define itp_window 3

# define IDLE 0
# define ON_MOVEL 1
# define ON_MOVEJ 2
# define ON_MOVE_FOLLOW 3
# define ON_CARMOVE 4


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

typedef struct
{
    unsigned int target_velocity;
    unsigned int ctrl_word;
    unsigned int mode_operation;
    unsigned int act_position;
    unsigned int DI;
    unsigned int act_velocity;
    unsigned int status_word;
} Offset_vel;

/* EtherCAT domain结构体  */
typedef struct 
{
    ec_domain_t *domain;
    ec_domain_state_t domain_state;
    uint8_t *domain_pd;
    std::vector<ec_pdo_entry_reg_t> domain_reg;
}EC_domain;

/* EtherCAT 地址结构体 */
typedef struct EC_position
{
    uint16_t alias;
    uint16_t buspos;
} EC_position;

/* 插值 结构体 */
typedef struct 
{
    double para[17];
    double time;
    double deltaTime;
}splan;

/* Elmo电机结构体 */
typedef struct 
{
    uint16_t alias;
    uint16_t buspos;
    Offset offset;
    uint16_t status;  /* DS402 status register, without manufacturer bits */
    ec_slave_config_t *sc_dig_out;

    uint8_t mode;       // 运行模式， 暂时只能固定为位置模式
    uint8_t first_time;

    int act_position; /* actual position */
    double exp_position;   /* expect position */
    double ref_position;
    uint32_t ain;       // 模拟输入2 
    double this_send;

    // s曲线插补
    splan sp;

    // 精插补
    std::vector<double> plan;
    double plan_param[4];
    uint16_t plan_cnt;
    double plan_run_time;
    uint16_t itp_period_times;

}Motor;

typedef struct 
{
    uint16_t alias;
    uint16_t buspos;
    Offset_vel offset;
    uint16_t status;  /* DS402 status register, without manufacturer bits */
    ec_slave_config_t *sc_dig_out;

    uint8_t mode;
    uint8_t first_time;

    int act_velocity; /* actual velocity in cnt */
    int act_position;
    double exp_velocity;   /* expect velocity in cnt */
    double ref_velocity;
    double this_send;

    splan sp;
    std::vector<double> plan;
    double plan_param[4];
    uint16_t plan_cnt;
    double plan_run_time;
    uint16_t itp_period_times;

}velMotor;

/* 力传感器 结构体 */
typedef struct 
{
    EC_position pos;
    Offset_ft offset;

    double ft[6];
    double offsetft[6];
    bool dataReady;
    double countsPerForce;
    double countsPerTorque;

    uint32_t status;
    uint32_t sampCount;
}ft_sensor;

typedef struct 
{
    int Switch;
    double totalP[6];
    double totalV[6];
    double totalTrans[16];
    double jointP[7];
    double paramM[6];
    double paramK[6];
    double paramC[6];
}forceCtrl;

/* 身体部件 结构体 */
typedef struct 
{
    double T00[16];
    Motor motor[7];
    uint8_t motornum;
    double jointPos[7];
    double jointGear[7];
    
    // ethercat domain 序号
    int dm_index;

    // s_plan 相关
    uint16_t plan_cnt;
    uint16_t itp_period_times;

    int s_planTimes;
    splan s_line;
    double rotInit[9];
    double locationInit[3];
    double locationDelta[3];
    splan s_beta;
    splan s_equat;
    double rEquivalent[4];

    // 力控
    ft_sensor endft;
    forceCtrl fctrl;

    // 重要状态位 
    int state;
    uint16_t movefollowCnt;

}bodypart;

typedef struct 
{
    velMotor motor[4];
    uint8_t motornum;
    int dm_index;       // ethercat domain 序号
    double jointGear[4];
    double jointVel[4];

    double chassisVel_cmd[2];

    uint16_t watchdog;
    uint16_t itp_period_times;

    int state;
}trackpart;


int FT_sensor_init(bodypart &arm, ec_master_t * m, int dm_index, EC_position pos);


# endif