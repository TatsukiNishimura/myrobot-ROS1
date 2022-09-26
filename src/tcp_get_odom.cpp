#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

void publish_odom(ros::Publisher &odom_pub, tf::TransformBroadcaster &odom_broadcaster, float &x, float &y,
                  float &yaw, float &vx, float &vy, float &omega);

std::array<float, 6> parser(uint8_t *rxBuf, ssize_t len);

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "tcp_get_odom");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 100);
    tf::TransformBroadcaster odom_broadcaster;

    int sockfd;
    int client_sockfd;
    struct sockaddr_in addr;
    socklen_t len = sizeof(struct sockaddr_in);
    struct sockaddr_in from_addr;

    uint8_t rxBuf[27] = {0};

    // ソケット生成
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("socket");
        exit(1);
    }

    // 送信先アドレス・ポート番号設定
    addr.sin_family = AF_INET;
    addr.sin_port = htons(6005);
    addr.sin_addr.s_addr = inet_addr("10.42.0.10"); // バインド

    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        exit(1);
    }

    // 受信待ち
    if (listen(sockfd, SOMAXCONN) < 0)
    {
        perror("listen");
        exit(1);
    }

    while (ros::ok())
    {
        printf("start\n");
        client_sockfd = accept(sockfd, (struct sockaddr *)&from_addr, &len);
        if (client_sockfd > 0)
        {
            while (recv(client_sockfd, rxBuf, sizeof(rxBuf), 0) && ros::ok())
            {
                std::array<float, 6> odom = parser(rxBuf, sizeof(rxBuf));
                for (int i = 0; i < 6; i++)
                {
                    printf("%f ", odom[i]);
                }
                printf("\n");

                publish_odom(odom_pub, odom_broadcaster, odom[0], odom[1],
                             odom[2], odom[3], odom[4], odom[5]);
            }
        }
    }
    close(client_sockfd);
    close(sockfd);

    return 0;
}

std::array<float, 6> parser(uint8_t *rxBuf, ssize_t len)
{
    std::array<float, 6> ret{0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    if (len == 27 && rxBuf[0] == 0xFF && rxBuf[1] == 0xFF && rxBuf[26] == 0xFE)
    {
        ret[0] = *reinterpret_cast<float *>(&rxBuf[2]);
        ret[1] = *reinterpret_cast<float *>(&rxBuf[6]);
        ret[2] = *reinterpret_cast<float *>(&rxBuf[10]);
        ret[3] = *reinterpret_cast<float *>(&rxBuf[14]);
        ret[4] = *reinterpret_cast<float *>(&rxBuf[18]);
        ret[5] = *reinterpret_cast<float *>(&rxBuf[22]);
    }
    return ret;
}

void publish_odom(ros::Publisher &odom_pub, tf::TransformBroadcaster &odom_broadcaster, float &x, float &y,
                  float &yaw, float &vx, float &vy, float &omega)
{
    ros::Time current_time = ros::Time::now();
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = omega;

    odom_pub.publish(odom);
}