#include "ros/ros.h"             // 包含ROS的头文件
#include "impedance/Impedance.h" // 包含阻抗控制器的头文件

int main(int argc, char **argv) // 主函数
{
    ros::init(argc, argv, "impedance_node"); // 初始化ROS节点，节点名为"impedance_node"

    ros::NodeHandle nh; // 创建ROS节点句柄

    // 参数
    double frequency; // 频率

    std::string topic_arm_state;    // 机械臂状态的话题名
    std::string topic_arm_command;  // 机械臂命令的话题名
    std::string topic_wrench_state; // 扭矩状态的话题名

    std::vector<double> desired_pose; // 期望姿态
    std::vector<double> Ka, Kv, Kp;   // 控制增益

    std::vector<double> M; // 质量
    std::vector<double> D; // 阻尼
    std::vector<double> K; // 刚度

    // 从ROS服务器加载参数

    // 话题名
    if (!nh.getParam("topic_arm_state", topic_arm_state))
    {
        ROS_ERROR("Couldn't retrieve the topic name for the state of the arm.");
        return -1;
    }
    if (!nh.getParam("topic_arm_command", topic_arm_command))
    {
        ROS_ERROR("Couldn't retrieve the topic name for commanding the arm.");
        return -1;
    }
    if (!nh.getParam("topic_wrench_state", topic_wrench_state))
    {
        ROS_ERROR("Couldn't retrieve the topic name for the force/torque sensor.");
        return -1;
    }

    // 控制增益
    if (!nh.getParam("Ka", Ka))
    {
        ROS_ERROR("Couldn't retrieve the Ka.");
        return -1;
    }
    if (!nh.getParam("Kv", Kv))
    {
        ROS_ERROR("Couldn't retrieve the Kv.");
        return -1;
    }
    if (!nh.getParam("Kp", Kp))
    {
        ROS_ERROR("Couldn't retrieve the Kp.");
        return -1;
    }

    // 物理参数
    if (!nh.getParam("mass_arm", M))
    {
        ROS_ERROR("Couldn't retrieve the desired mass of the arm.");
        return -1;
    }
    if (!nh.getParam("damping_arm", D))
    {
        ROS_ERROR("Couldn't retrieve the desired damping of the coupling.");
        return -1;
    }
    if (!nh.getParam("stiffness_coupling", K))
    {
        ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling.");
        return -1;
    }

    // 期望姿态
    if (!nh.getParam("desired_pose", desired_pose))
    {
        ROS_ERROR("Couldn't retrieve the desired_pose.");
        return -1;
    }

    Impedance impedance; // 创建阻抗控制器对象

    impedance.init( // 初始化阻抗控制器
        nh,
        topic_arm_state,
        topic_arm_command,
        topic_wrench_state,
        Ka, Kv, Kp,
        M, D, K,
        desired_pose);

    impedance.run(); // 运行阻抗控制器

    return 0; // 退出程序
}