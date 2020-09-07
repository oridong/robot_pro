# ifndef __COMU__

#define __COMU__

#include <regex.h>
#include <sys/types.h>

# include "control.h"
/******************* 网络参数 *******************/

# define CTRL_IP "10.8.0.2" 
# define CTRL_PORT 8001

# define Nvidia_IP "10.8.0.4" 
# define Nvidia_PORT 8002

# define ROBOT_IP "10.8.0.3" //"10.1.76.247"//
# define ROBOT_PORT 8000

# define RECV_BUF_SIZE 512  // 接收缓存区大小
# define CMD_SIZE 30        // 命令字大小
# define PARAM_SIZE 200     // 后跟随参数大小
# define PARAM_LIST_SIZE 20 // 每个参数大小

# define HEAD_MOVE 0
# define CAR_MOVE 1
# define STOP_ALL 2
# define ENABLE_MOTOR 3
# define DISABLE_MOTOR 4
# define MOVEJ 5
# define MOVEL 6
# define MOVE_FOLLOW 7

# define NO_RECV -2
# define ERROR_MATCH -1


const char cmd_table[][CMD_SIZE] = {
    "headMove",
    "carmove",
    "stopAll",
    "EnMotor",
    "DisMotor",
    "moveJ",
    "moveL",
    "moveFollow"
};

typedef struct{
    int cmd_mode;
    char param_list[PARAM_LIST_SIZE][10];
    int param_cnt;

}command;

static regex_t reg ;

int UDP_init(void);
command robotReceiveCommand(void);
void robotSendFeedback(bodypart la, bodypart ra, bodypart head, bodypart track);




#endif