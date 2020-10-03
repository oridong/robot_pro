#include "communication.h"

int sockfd;
struct sockaddr_in ctrl_addr;
struct sockaddr_in received_addr;
struct sockaddr_in nvidia_addr;
command cmd;
char error_buf[64];
static const char *pattern = "^(\\w+)\\((.*)\\)$";

char recv_buf[RECV_BUF_SIZE] = "";
char param_str[PARAM_SIZE] = "";        
char cmd_str[CMD_SIZE] = "";

static uint8_t state_cnt = 0;

regmatch_t pm[10];

int UDP_init()
{

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) // 创建套接字
    {
        perror("socket");
        exit(-1);
    }

    // 本地网络地址
    struct sockaddr_in host_addr;
    bzero(&host_addr, sizeof(host_addr));            // 清空结构体内容
    host_addr.sin_family = AF_INET;                  // ipv4
    host_addr.sin_port = htons(ROBOT_PORT);          // 端口转换
    host_addr.sin_addr.s_addr = inet_addr(ROBOT_IP); // 绑定网卡所有ip地址，INADDR_ANY为通配地址，值为0

    printf("Binding server to Robot %s:%d\n", ROBOT_IP, ROBOT_PORT);
    if (bind(sockfd, (struct sockaddr *)&host_addr, sizeof(host_addr)) != 0)
    {
        perror("bind error");
        close(sockfd);
        exit(-1);
    }

    ctrl_addr.sin_family = AF_INET;
    ctrl_addr.sin_port = htons(CTRL_PORT);
    ctrl_addr.sin_addr.s_addr = inet_addr(CTRL_IP);

    nvidia_addr.sin_family = AF_INET;
    nvidia_addr.sin_port = htons(Nvidia_PORT);
    nvidia_addr.sin_addr.s_addr = inet_addr(Nvidia_IP);

    int z = 0;
    z = regcomp(&reg, pattern, REG_EXTENDED);

    if (z != 0)
    {
        regerror(z, &reg, error_buf, sizeof(error_buf));
        fprintf(stderr, "%s: pattern '%s' \n", error_buf, pattern);
        return -1;
    }
    printf("RegEx init Successfully\n");

    return 0;
}

command robotReceiveCommand(void)
{
    int recv_len;
    char cli_ip[INET_ADDRSTRLEN] = ""; //INET_ADDRSTRLEN=16
    socklen_t rcvaddr_len = sizeof(received_addr);
    command cmd;

    const size_t nmatch = 3;
    uint8_t match_cnt = 0;
    uint8_t param_cnt = 0;

    int z;
    char *token;

    // 接受数据
    memset(recv_buf, 0, sizeof(recv_buf));
    recv_len = recvfrom(sockfd, recv_buf, sizeof(recv_buf), MSG_DONTWAIT, (struct sockaddr *)&received_addr, &rcvaddr_len);

    // inet_ntop(AF_INET, &ctrl_addr.sin_addr, cli_ip, INET_ADDRSTRLEN);   // 转换IP为字符串接口，INET_ADDRSTRLEN为地址长度，为16字符
    // printf("\nip:%s ,port:%d\n",cli_ip, ntohs(ctrl_addr.sin_port));
    if (recv_len > 0)
    {
        printf("received data(%d):%s\n", recv_len, recv_buf);

        z = strlen(recv_buf);

        if (z > 0 && recv_buf[z - 1] == '\n')
        {
            recv_buf[z - 1] = 0;
        }

        /* 对每一行应用正则表达式进行匹配 */
        
        z = regexec(&reg, recv_buf, nmatch, pm, 0);
        if (z == REG_NOMATCH) // success = 0
        {
            printf("nomatch\n");
            cmd.cmd_mode = -1;
            return cmd;
        }
        else if (z != 0)
        {
            regerror(z, &reg, error_buf, sizeof(error_buf));
            fprintf(stderr, "%s: regcom('%s')\n", error_buf, recv_buf);
            cmd.cmd_mode = -1;
            return cmd;
        }

        memset(cmd_str, 0, sizeof(cmd_str));
        memset(param_str, 0, sizeof(param_str));
        memset(cmd.param_list, 0, sizeof(cmd.param_list));

        // 循环每条匹配到的子串
        for (match_cnt = 0; match_cnt < nmatch && pm[match_cnt].rm_so != -1; ++match_cnt)
        {

            // 第0条为原输入串， 第1条为函数名， 第2条为参数列表
            if (match_cnt == 1)
            {
                strncpy(cmd_str, recv_buf + pm[match_cnt].rm_so, pm[match_cnt].rm_eo - pm[match_cnt].rm_so);
                // printf("cmd = %s\n", cmd_str);
            }
            if (match_cnt == 2)
            {
                strncpy(param_str, recv_buf + pm[match_cnt].rm_so, pm[match_cnt].rm_eo - pm[match_cnt].rm_so);

                // 对参数列表进行逗号分割
                cmd.param_cnt = 0;
                char *token = strtok(param_str, ",");

                for (cmd.param_cnt = 0; cmd.param_cnt < PARAM_LIST_SIZE && token != NULL; ++cmd.param_cnt)
                {
                    memcpy(cmd.param_list[cmd.param_cnt], token, sizeof(token));
                    token = strtok(NULL, ",");
                    // printf("param%d:%s\n", cmd.param_cnt, cmd.param_list[cmd.param_cnt]);
                }
            }
        }
       
        z = 0;
        while ( z < sizeof(cmd_table)/CMD_SIZE)
        {
            if (!strcmp(cmd_table[z], cmd_str))
            {
                cmd.cmd_mode = z;
                break;
            }
            else
            {
                cmd.cmd_mode = -1;
            }
            z ++;
        }
        
        //printf("match success %d \n", cmd.cmd_mode);
        return cmd;
        

    }
    else
    {
        cmd.cmd_mode = -2;
        return cmd;
    }
    
}

void robotSendFeedback(bodypart la, bodypart ra, bodypart head, bodypart leg, trackpart trc)
{
    char send_buf[2048] = "";
    sprintf(send_buf, " %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,\n  ", 
    la.motor[0].act_position/la.jointGear[0], 
    la.motor[1].act_position/la.jointGear[1],la.motor[2].act_position/la.jointGear[2],
    la.motor[3].act_position/la.jointGear[3],la.motor[4].act_position/la.jointGear[4],
    la.motor[5].act_position/la.jointGear[5],la.motor[6].act_position/la.jointGear[6], 
    ra.motor[0].act_position/ra.jointGear[0],
    ra.motor[1].act_position/ra.jointGear[1],ra.motor[2].act_position/ra.jointGear[2],
    ra.motor[3].act_position/ra.jointGear[3],ra.motor[4].act_position/ra.jointGear[4],
    ra.motor[5].act_position/ra.jointGear[5],ra.motor[6].act_position/ra.jointGear[6], 
    
    head.motor[0].act_position/head.jointGear[0], 
    head.motor[1].act_position/head.jointGear[1], 
    head.motor[2].act_position/head.jointGear[2], 
    leg.motor[0].act_position/leg.jointGear[0],
    leg.motor[2].act_position/leg.jointGear[2],
    leg.motor[1].act_position/leg.jointGear[1],
    leg.motor[3].act_position/leg.jointGear[3],
    leg.motor[4].act_position/leg.jointGear[4]
    );
    send_buf[strlen(send_buf) - 1] = '\0';

    // printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,\n  ", 
    // la.motor[0].this_send/la.jointGear[0], 
    // la.motor[1].this_send/la.jointGear[1],la.motor[2].this_send/la.jointGear[2],
    // la.motor[3].this_send/la.jointGear[3],la.motor[4].this_send/la.jointGear[4],
    // la.motor[5].this_send/la.jointGear[5],la.motor[6].this_send/la.jointGear[6], 
    // ra.motor[0].this_send/ra.jointGear[0],
    // ra.motor[1].this_send/ra.jointGear[1],ra.motor[2].this_send/ra.jointGear[2],
    // ra.motor[3].this_send/ra.jointGear[3],ra.motor[4].this_send/ra.jointGear[4],
    // ra.motor[5].this_send/ra.jointGear[5],ra.motor[6].this_send/ra.jointGear[6], 
    
    // head.motor[0].this_send/head.jointGear[0], 
    // head.motor[1].this_send/head.jointGear[1], 
    // head.motor[2].this_send/head.jointGear[2], 
    // la.endft.ft[2]);

    char motorstate_buf[100] = "";
    uint32_t mstate = 0;
    int i = 0, j = 0;

    mstate |= !(la.fctrl.Switch == 0);
    mstate |= !(ra.fctrl.Switch == 0) << 1;

    i = 2;
    for (j = 0; j < 7; j++, i ++)
    {
        if (la.motor_use[j] == 1)
            mstate |= (la.motor[j].servo_state == 1) << i;
    }

    for (j = 0; j < 7; j++, i ++)
    {
        if (ra.motor_use[j] == 1)
            mstate |= (ra.motor[j].servo_state == 1) << i;
    }

    for (j = 0; j < 3; j++, i ++)
    {
        if (head.motor_use[j] == 1)
            mstate |= (head.motor[j].servo_state == 1) << i;
    }

    for (j = 0; j < 5; j++, i ++)
    {
        if (leg.motor_use[j] == 1)
            mstate |= (leg.motor[j].servo_state == 1) << i;
    }

    for (j = 0; j < 4; j++, i ++)
    {
        if (trc.motor_use[j] == 1)
        mstate |= (trc.motor[j].servo_state == 1) << i;
    }

    mstate |= (1 == 1) <<(i++);        //tool
    mstate |= (1 == 1) <<(i++);        //tool

    if (mstate == (1<<i) - 1)
    {
        mstate |= 1 << i;
    }
    else
    {
        mstate |= 0 << i;
    }

    sprintf(motorstate_buf, "%d", mstate);
    // printf("%x, %d\n", mstate, i);
    
    //发送数据
    int len;
    len = sendto(sockfd, send_buf, strlen(send_buf), 0, (struct sockaddr *)&nvidia_addr, sizeof(nvidia_addr));
    
    if (state_cnt > 200){
        state_cnt = 0;
        len = sendto(sockfd, motorstate_buf, strlen(motorstate_buf), 0, (struct sockaddr *)&ctrl_addr, sizeof(ctrl_addr));
    }
    state_cnt ++;
    // printf("len = %d\n", len);
}
