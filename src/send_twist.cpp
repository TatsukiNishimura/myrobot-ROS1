#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <mutex>

std::mutex m;

geometry_msgs::Twist global_twist;
void twistCallback(const geometry_msgs::Twist &twist);

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "send_twist_node");
    ros::NodeHandle nh;
    ros::Subscriber sub;
    sub = nh.subscribe("cmd_vel", 10, twistCallback);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(50);

    int sockfd;
    struct sockaddr_in addr;

    uint8_t rxBuf[30] = {0};

    // ソケット生成
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("socket");
    }

    // 送信先アドレス・ポート番号設定
    addr.sin_family = AF_INET;
    addr.sin_port = htons(80);
    addr.sin_addr.s_addr = inet_addr("10.42.0.2");

    // サーバ接続

    while (ros::ok())
    {
        uint8_t data[15] = {0};
        data[0] = 0xFF;
        data[1] = 0xFF;
        float vx = 0.f;
        float vy = 0.f;
        float omega = 0.f;
        {
            std::lock_guard<std::mutex> lock(m);
            vx = global_twist.linear.x;
            vy = global_twist.linear.y;
            omega = global_twist.angular.z;
        }
        memcpy(&data[2], &vx, sizeof(float));
        memcpy(&data[6], &vy, sizeof(float));
        memcpy(&data[10], &omega, sizeof(float));
        data[14] = 0xFE;

        if (send(sockfd, data, sizeof(data), 0) > 0)
        {
            printf("sent : ");
            for (int i = 0; i < sizeof(data); i++)
            {
                printf("%d ", data[i]);
            }
            printf("\n");
        }
        else
        {
            close(sockfd);
            sockfd = socket(AF_INET, SOCK_STREAM, 0);
            printf("error");
            printf("\n");
            connect(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
            // sleep(1);
        }

        loop_rate.sleep();
    }
    spinner.stop();
    // ソケットクローズ
    close(sockfd);

    return 0;
}

void twistCallback(const geometry_msgs::Twist &twist)
{
    std::lock_guard<std::mutex> lock(m);
    global_twist = twist;
}
