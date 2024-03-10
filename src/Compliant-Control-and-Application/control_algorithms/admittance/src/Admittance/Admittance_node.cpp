/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:08:30 
 * @Last Modified by:   MingshanHe 
 * @Last Modified time: 2021-12-05 04:08:30 
 * @Licence: MIT Licence
 */
// 引入ROS核心头文件和Admittance控制器的定义
#include "ros/ros.h"
#include "admittance/Admittance.h"

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "admittance_node");

    // 创建节点句柄
    ros::NodeHandle nh;
    // 控制循环的频率
    double frequency = 100.0;

    // 参数
    std::string topic_arm_state; // 机械臂状态的主题名
    std::string topic_arm_command; // 发送给机械臂的命令的主题名
    std::string topic_wrench_state; // 力/扭矩传感器状态的主题名
    std::string base_link; // 基础连接点的名称
    std::string end_link; // 末端连接点的名称

    // 顺应性控制参数
    std::vector<double> M; // 质量参数
    std::vector<double> D; // 阻尼参数
    std::vector<double> K; // 刚度参数
    std::vector<double> desired_pose; // 期望姿态
    
    double arm_max_vel; // 机械臂最大速度
    double arm_max_acc; // 机械臂最大加速度

    // 从ROS参数服务器加载参数

    // 主题名称
    if (!nh.getParam("topic_arm_state", topic_arm_state)) { ROS_ERROR("Couldn't retrieve the topic name for the state of the arm."); return -1; }
    if (!nh.getParam("topic_arm_command", topic_arm_command)) { ROS_ERROR("Couldn't retrieve the topic name for commanding the arm."); return -1; }
    if (!nh.getParam("topic_wrench_state", topic_wrench_state)) { ROS_ERROR("Couldn't retrieve the topic name for the force/torque sensor."); return -1; }
    // 顺应性参数
    if (!nh.getParam("mass_arm", M)) { ROS_ERROR("Couldn't retrieve the desired mass of the arm."); return -1; }
    if (!nh.getParam("damping_arm", D)) { ROS_ERROR("Couldn't retrieve the desired damping of the coupling."); return -1; }
    if (!nh.getParam("stiffness_coupling", K)) { ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling."); return -1; }
    if (!nh.getParam("base_link", base_link)) { ROS_ERROR("Couldn't retrieve the base_link."); return -1; }
    if (!nh.getParam("end_link", end_link)) { ROS_ERROR("Couldn't retrieve the end_link."); return -1; } 
    if (!nh.getParam("desired_pose", desired_pose)) { ROS_ERROR("Couldn't retrieve the desired pose of the spring."); return -1; }
    if (!nh.getParam("arm_max_vel", arm_max_vel)) { ROS_ERROR("Couldn't retrieve the max velocity for the arm."); return -1;}
    if (!nh.getParam("arm_max_acc", arm_max_acc)) { ROS_ERROR("Couldn't retrieve the max acceleration for the arm."); return -1;}
    // 构造控制器
    Admittance admittance(
        nh,
        frequency,
        topic_arm_state,
        topic_arm_command,
        topic_wrench_state,
        M, D, K, desired_pose,
        base_link,
        end_link,
        arm_max_vel,
        arm_max_acc);

    // 运行控制器
    admittance.run();

    return 0;
}