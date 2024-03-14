#ifndef IMPEDACE_H  // 如果未定义IMPEDACE_H
#define IMPEDACE_H  // 定义IMPEDACE_H

// 包含必要的头文件
#include <ros/node_handle.h>  // ROS节点句柄
#include <hardware_interface/joint_command_interface.h>  // 硬件接口中的关节命令接口
#include <controller_interface/controller.h>  // 控制器接口
#include <kdl/chainiksolvervel_pinv.hpp>  // KDL的逆速度解算器（伪逆）
#include <kdl/chainiksolvervel_pinv_givens.hpp>  // KDL的逆速度解算器（Givens旋转）
#include <kdl/chainfksolvervel_recursive.hpp>  // KDL的正速度解算器
#include <kdl/chainfksolverpos_recursive.hpp>  // KDL的正位置解算器
#include <kdl/chainiksolverpos_nr.hpp>  // KDL的逆位置解算器（Newton-Raphson）
#include <realtime_tools/realtime_publisher.h>  // 实时发布工具
#include "KDL_Base.h"  // KDL基础类
#include <kdl/chainidsolver_recursive_newton_euler.hpp>  // KDL的逆动力学解算器
#include <kdl/chaindynparam.hpp>  // KDL的动力学参数

#include "Eigen/Core"  // Eigen库的核心部分
#include "Eigen/Geometry"  // Eigen库的几何部分
#include "Eigen/Dense"  // Eigen库的密集矩阵部分
#include <eigen_conversions/eigen_msg.h>  // Eigen类型与ROS消息类型之间的转换
#include <geometry_msgs/Twist.h>  // ROS的Twist消息类型
#include <geometry_msgs/Pose.h>  // ROS的Pose消息类型
#include <std_msgs/Float64.h>  // ROS的Float64消息类型
#include "joint_effort_msg/JointEffort.h"  // 自定义的关节努力消息
#include "joint_effort_msg/JointEfforts.h"  // 自定义的关节努力消息组
#include "joint_state_msg/JointState.h"  // 自定义的关节状态消息
#include "geometry_msgs/Wrench.h"  // ROS的Wrench消息类型
#include "std_msgs/Float64MultiArray.h"  // ROS的Float64多维数组消息类型

typedef Eigen::Matrix<double, 6, 6> Matrix6d;  // 定义一个6x6的double类型矩阵

// 声明Impedance类，继承自kdl_base::KDL_Base
class Impedance: public kdl_base::KDL_Base
{
public:
    Impedance() {}  // 构造函数
    ~Impedance() {}  // 析构函数

    // 初始化函数声明
    void init(ros::NodeHandle &nh,
        std::string topic_arm_state,
        std::string topic_arm_command,
        std::string topic_wrench_state,
        std::vector<double> Ka,
        std::vector<double> Kv,
        std::vector<double> Kp,
        std::vector<double> M,
        std::vector<double> D,
        std::vector<double> K,
        std::vector<double> desired_pose);

    void run();  // 运行控制器的函数声明

private:

    void compute_impedance(bool flag);  // 计算阻抗的函数声明

private:
    void state_arm_callback(const joint_state_msg::JointState msg);  // 处理关节状态消息的回调函数

    void state_wrench_callback(const geometry_msgs::WrenchConstPtr msg);  // 处理扭矩状态消息的回调函数

    void command(const std_msgs::Float64MultiArray::ConstPtr &msg);  // 处理控制命令消息的回调函数

    void send_commands_to_robot();  // 将控制命令发送给机器人的函数声明

protected:
    ros::NodeHandle                 nh_;  // ROS节点句柄
    // 订阅器:
    ros::Subscriber                 sub_command_; // 订阅外部命令的接口
    ros::Subscriber                 sub_arm_state_;  // 订阅机械臂状态的接口
    ros::Subscriber                 sub_wrench_state_;  // 订阅扭矩状态的接口
    ros::Subscriber                 sub_posture_;  // 订阅姿态的接口
    // 发布器:
    ros::Publisher                  pub_arm_cmd_;  // 发布机械臂命令的接口

    ros::Time                       last_publish_time_;  // 上一次发布的时间
    double                          publish_rate_;  // 发布率

    // KDL变量:
    KDL::JntArray                   Jnt_Pos_Init_State;  // 关节的初始位置状态
    KDL::JntArrayAcc                Jnt_Desired_State;  // 期望的关节状态（包含加速度信息）
    KDL::JntArray                   CMD_State;  // 命令状态
    KDL::JntArray                   Current_State;  // 当前状态
    KDL::JntArray                   Jnt_Toq_Cmd_;  // 关节扭矩命令

    KDL::JntArray                   Jnt_Pos_State;  // 关节位置状态
    KDL::JntArray                   Jnt_Vel_State;  // 关节速度状态
    KDL::JntArray                   Jnt_Toq_State;  // 关节扭矩状态

    KDL::Wrenches                   Ext_Wrenches;  // 外部扭矩

    KDL::Rotation                   Desired_Ori_;  // 期望的方向
    KDL::Vector                     Desired_Pos_;  // 期望的位置
    KDL::Frame                      Desired_Pose_;  // 期望的姿态

    KDL::Vector                     Gravity;  // 重力向量

    KDL::JntSpaceInertiaMatrix      M_; // 惯性矩阵
    KDL::JntArray                   C_, G_;  // C为科氏力向量，G为重力向量
    KDL::JntArray                   Kp_, Kv_, Ka_;  // 分别为位置、速度、加速度控制的增益

    bool                            Recieved_Joint_State;  // 是否收到了关节状态
    bool                            Cmd_Flag_;  // 命令标志
    bool                            Init_Flag_;  // 初始化标志
    uint                            Step_;  // 步骤计数

    // 运动学求解器
    boost::shared_ptr<KDL::ChainFkSolverPos>    fk_pos_solver_;  // 正向位置运动学求解器
    boost::shared_ptr<KDL::ChainFkSolverVel>    fk_vel_solver_;  // 正向速度运动学求解器
    boost::shared_ptr<KDL::ChainIkSolverVel>    ik_vel_solver_;  // 逆向速度运动学求解器
    boost::shared_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;  // 逆向位置运动学求解器

    // 动力学求解器
    boost::shared_ptr<KDL::ChainIdSolver>       id_pos_solver_;  // 逆向动力学位置求解器
    boost::shared_ptr<KDL::ChainDynParam>       id_solver_;  // 动力学参数求解器

    std::vector<double>     Impedance_M, Impedance_D, Impedance_K;  // 阻抗控制的质量、阻尼、刚度参数
    std::vector<double>     desired_pose_;  // 期望的姿态

    double                  wrench_x;  // X方向的扭矩
    double                  wrench_y;  // Y方向的扭矩
    double                  wrench_z;  // Z方向的扭矩
    double                  pos_x;  // X方向的位置
    double                  pos_y;  // Y方向的位置
    double                  pos_z;  // Z方向的位置
};

#endif
