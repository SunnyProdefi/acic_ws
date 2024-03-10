/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:08:58 
 * @Last Modified by: MingshanHe
 * @Last Modified time: 2021-12-05 04:13:10
 * @Licence: MIT Licence
 */
// 包含ROS核心头文件和用于发布力矩信息的消息类型
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

// 定义发布力矩信息的主题名和发布频率
#define WRENCH_TOPIC    "/wrench_fake"
#define TOPIC_HZ        125.0

int main(int argc, char ** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "wrench_signal_generate");
    // 创建节点句柄
    ros::NodeHandle nh;

    // 定义一个发布者，用于发布力矩信息，队列大小为5
    ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>(WRENCH_TOPIC, 5);
    // 程序开始后先暂停5秒，确保ROS系统连接稳定
    ros::Duration Sleep(5.0);
    Sleep.sleep();
    // 定义循环率，根据前面定义的频率来设置
    ros::Rate   loop_rate(TOPIC_HZ);
    // 初始化力矩消息
    geometry_msgs::WrenchStamped wrench_msg;
    // 时间变量，用于计算力的变化
    double t = 0;
    while (ros::ok())
    {
        // 根据时间t来周期性改变力的z分量，前5秒为1，后5秒为-1，循环变化
        if(static_cast<int>(t)%10 < 5)
        {
            wrench_msg.wrench.force.z = 1;
        }
        else
        {
            wrench_msg.wrench.force.z = -1;
        }
        // 时间增加，步长为1除以发布频率
        t += 1/TOPIC_HZ;
        // 发布力矩信息
        wrench_pub.publish(wrench_msg);
        // 按照设定的频率休眠，确保以固定频率发布信息
        loop_rate.sleep();
    }
    
}
