#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

std::vector<uint8_t> read_until(int fd, char del);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "get_odometry_node");
    ros::NodeHandle n;
    uint8_t buffer[10] = {0};
    const char *device_name = "/dev/ttyACM0";
    int fd1 = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd1, F_SETFL, 0);
    // load configuration
    struct termios conf_tio;
    tcgetattr(fd1, &conf_tio);
    // set baudrate
    speed_t BAUDRATE = B1000000;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);
    // non canonical, non echo back
    conf_tio.c_lflag &= ~(ECHO | ICANON);
    // non blocking
    conf_tio.c_cc[VMIN] = 0;
    conf_tio.c_cc[VTIME] = 0;
    // store configuration
    tcsetattr(fd1, TCSANOW, &conf_tio);

    std::vector<uint8_t> bytesArray;
    uint8_t state = 0;
    uint8_t count = 0;
    ros::Rate loop_rate(200);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 50);
    tf::TransformBroadcaster odom_broadcaster;
    while (ros::ok())
    {
        bytesArray.clear();
        bytesArray = read_until(fd1, '\n');
        std::string buf;
        std::vector<float> data;
        for (int i = 0; i < bytesArray.size(); i++)
        {
            // printf("%c ", bytesArray[i]);
            if (bytesArray[i] == ',')
            {
                try
                {
                    const float num = std::stof(buf);
                    data.emplace_back(num);
                    // printf("%f ", num);
                    buf.clear();
                }
                catch (const std::invalid_argument &e)
                {
                    std::cout << "invalid argument" << std::endl;
                }
                catch (const std::out_of_range &e)
                {
                    std::cout << "out of range" << std::endl;
                }
            }
            else if (bytesArray[i] == '\n')
            {
                buf.clear();
                // printf("\n");
            }
            else
            {
                buf.push_back(bytesArray[i]);
            }
        }
        if (data.size() != 6)
        {
            continue;
        }
        const float x = data[0];
        const float y = data[1];
        const float yaw = data[2];
        const float vx = data[3];
        const float vy = data[4];
        const float omega = data[5];
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
        printf("x : %10.6f y : %10.6f yaw : %10.6f vx : %10.6f vy : %10.6f omega : %10.6f\r\n", x, y, yaw, vx, vy, omega);
        loop_rate.sleep();
    }

    close(fd1);
    return 0;
}

std::vector<uint8_t> read_until(int fd, char del)
{
    char c = 0;
    std::vector<uint8_t> ret;
    while (1)
    {
        if (read(fd, &c, 1))
        {
            ret.emplace_back(c);
        }
        if (c == del)
        {
            break;
        }
    }
    return ret;
}
