# ifndef __COMU__

#define __COMU__

#include <regex.h>
#include <sys/types.h>

# include "control.h"
/******************* 网络参数 *******************/

# define CTRL_IP "10.1.77.181"
# define CTRL_PORT 8001

# define ROBOT_IP "10.1.76.247"
# define ROBOT_PORT 8000

# define RECV_BUF_SIZE 512
# define CMD_SIZE 30
# define PARAM_SIZE 200
# define PARAM_LIST_SIZE 20

# define HEAD_MOVE 0
# define CAR_MOVE 1
# define STOP_ALL 2
# define ENABLE_MOTOR 3

# define NO_RECV -2
# define ERROR_MATCH -1


const char cmd_table[][CMD_SIZE] = {
    "headMove",
    "carmove",
    "stopAll"
    "EnMotor"
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