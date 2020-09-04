#include "communication.h"

int sockfd;
struct sockaddr_in ctrl_addr;
command cmd;
char error_buf[64];
static const char *pattern = "^(\\w+):(.*)$";

char recv_buf[RECV_BUF_SIZE] = "";
char param_str[PARAM_SIZE] = "";
char cmd_str[CMD_SIZE] = "";

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
    socklen_t cliaddr_len = sizeof(ctrl_addr);
    command cmd;

    const size_t nmatch = 3;
    uint8_t match_cnt = 0;
    uint8_t param_cnt = 0;

    int z;
    char *token;

    // 接受数据
    memset(recv_buf, 0, sizeof(recv_buf));
    recv_len = recvfrom(sockfd, recv_buf, sizeof(recv_buf), MSG_DONTWAIT, (struct sockaddr *)&ctrl_addr, &cliaddr_len);

    // inet_ntop(AF_INET, &ctrl_addr.sin_addr, cli_ip, INET_ADDRSTRLEN);   // 转换IP为字符串接口，INET_ADDRSTRLEN为地址长度，为16字符
    // printf("\nip:%s ,port:%d\n",cli_ip, ntohs(ctrl_addr.sin_port));
    if (recv_len > 0)
    {
        printf("received data(%d):%s\n", recv_len, recv_buf);

        z = strlen(recv_buf);

        // token = strtok(recv_buf, "\n");

        // for (cmd.param_cnt = 0; cmd.param_cnt < 10 && token != NULL; ++cmd.param_cnt)
        // {
        //     memcpy(cmd.param_list[cmd.param_cnt], token, sizeof(token));
        //     token = strtok(NULL, "\n");
        //     // printf("param%d:%s\n", cmd.param_cnt, cmd.param_list[cmd.param_cnt]);
        // }

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
        
        printf("match success %d \n", cmd.cmd_mode);
        return cmd;

    }
    else
    {
        cmd.cmd_mode = -2;
        return cmd;
    }
    
}

void robotSendFeedback(Motor m[])
{
    char send_buf[1024] = "";
    sprintf(send_buf, "d0:%d,%d,%d\n ", m[0].act_position,m[0].exp_position , m[0].ain);
    send_buf[strlen(send_buf) - 1] = '\0';
    //发送数据
    int len = sendto(sockfd, send_buf, strlen(send_buf), 0, (struct sockaddr *)&ctrl_addr, sizeof(ctrl_addr));
    // printf("len = %d\n", len);
}
