#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "desired_pose_publisher");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建发布者，发布到desired_pose_update主题，队列大小设置为10
    ros::Publisher desired_pose_pub = nh.advertise<geometry_msgs::Pose>("/desired_pose_update", 10);

    // 设置循环频率
    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok())
    {
        // 创建一个Pose消息
        geometry_msgs::Pose updated_pose;

        // 设置新的期望姿态，这里仅作为示例
        updated_pose.position.x = 0.5;
        updated_pose.position.y = 0.0;
        updated_pose.position.z = 0.5;
        updated_pose.orientation.x = 0.0;
        updated_pose.orientation.y = 0.0;
        updated_pose.orientation.z = 0.0;
        updated_pose.orientation.w = 1.0;

        // 发布更新的desired_pose
        desired_pose_pub.publish(updated_pose);

        ROS_INFO("Published new desired_pose");

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}