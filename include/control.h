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


// 机械臂状态机， IDLE状态有至少一个电机使能，如果没有电机使能将会跳转到DISABLE状态
// 开始运动之后机械臂将会处于ON_MOVEJ，ON_MOVEJ，ON_MOVE_FOLLOW状态之一
// 履带会处于ON_CARMOVE状态
# define DISABLE -1
# define IDLE 0
# define ON_MOVEL 1
# define ON_MOVEJ 2
# define ON_MOVE_FOLLOW 3
# define ON_CARMOVE 4
# define ON_MOVETEST 5

/* 位置模式电机EtherCAT，Pdo地址偏移  */
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
    unsigned int current;
} Offset;

/* 力传感器EtherCAT，Pdo地址偏移  */
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

/* 速度模式电机EtherCAT，Pdo地址偏移  */
typedef struct
{
    unsigned int target_velocity;
    unsigned int ctrl_word;
    unsigned int mode_operation;
    unsigned int act_position;
    unsigned int DI;
    unsigned int act_velocity;
    unsigned int status_word;
    unsigned int current;
} Offset_vel;


/* EtherCAT domain结构体， 方便组装domain*/
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

/* s曲线规划插值 结构体 */
typedef struct 
{
    double para[17];
    double time;
    double deltaTime;
}splan;

/* Elmo位置模式电机结构体 */
typedef struct 
{
    uint16_t alias;     // ethercat总线别名
    uint16_t buspos;    // ethercat总线地址
    Offset offset;      // ethercat Pdo偏移
    uint16_t status;    // ethercat状态字
    ec_slave_config_t *sc_dig_out;      // ethercat从站配置

    // 重要状态量
    uint8_t mode;       // 运行模式，暂时只能固定为位置模式
    uint16_t servo_state;        // 运行模式，暂时只能固定为位置模式
    uint8_t servo_cmd;
    uint8_t first_time;

    // 电机变量的所有值均为cnt单位
    int act_position; /* actual position */
    double act_current; /* actual current */
    int start_pos;
    double exp_position;   /* expect position */
    double ref_position;
    double this_send;

    // 模拟量为4096格数单位
    uint32_t ain;       // 模拟输入2 

    // s曲线插补
    splan sp;

    // 电机位置精插补
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
    uint8_t servo_cmd;
    uint8_t servo_state;

    int act_velocity; /* actual velocity in cnt */
    int act_position;
    double act_current; /* actual current */
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

// 力控制参数
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
    double gearRatio[7];
    double startJointAngle[7];
    double offsetAngle[7];
    
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

    // 测试
    uint8_t motor_use[7];
    double test_time;
    double test_A;
    double test_T;

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

    // 测试
    uint8_t motor_use[4];

    int state;
}trackpart;


int FT_sensor_init(bodypart &arm, ec_master_t * m, int dm_index, EC_position pos);
void readForceData(bodypart &arm);
void check_follow(bodypart & arm, double timeout);


# endif