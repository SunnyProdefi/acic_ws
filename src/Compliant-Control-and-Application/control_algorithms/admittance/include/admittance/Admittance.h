/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:08:00 
 * @Last Modified by: MingshanHe
 * @Last Modified time: 2021-12-05 04:08:21
 * @Licence: MIT Licence
 */

// 防止重复包含头文件
#ifndef ADMITTANCE_H
#define ADMITTANCE_H

// 引入ROS和相关消息类型的头文件
#include "ros/ros.h"

#include "cartesian_state_msgs/PoseTwist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

// 引入C++标准库和Eigen数学库的一些常用功能
#include <memory>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <cmath>

// 使用Eigen命名空间中的所有名称
using namespace Eigen;

// 定义了7维和6维的向量以及6x6的矩阵，用于矩阵运算
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
// 定义圆周率常量
#define PI 3.1415926

// Admittance类定义
class Admittance
{
protected:
  // ROS变量:
  ros::NodeHandle nh_; // ROS节点句柄
  ros::Rate loop_rate_; // 循环频率

  // 顺应性控制参数:
  Matrix6d M_, D_, K_; // 质量矩阵，阻尼矩阵，刚度矩阵

  // 订阅器:
  ros::Subscriber sub_arm_state_; // 机械臂状态订阅器
  ros::Subscriber sub_wrench_state_; // 外力状态订阅器
  // 发布器:
  ros::Publisher pub_arm_cmd_; // 机械臂命令发布器

  // 变量:
  Vector3d      arm_position_; // 机械臂位置
  Quaterniond   arm_orientation_; // 机械臂方向
  Vector6d      arm_twist_; // 机械臂扭力
  Vector6d      wrench_external_; // 外部力矩
  Vector6d      arm_desired_twist_adm_; // 顺应性控制下机械臂的期望扭力
  Vector6d      arm_desired_accelaration; // 机械臂期望加速度

  Vector7d      desired_pose_; // 期望姿态
  Vector3d      desired_pose_position_; // 期望姿态位置
  Quaterniond   desired_pose_orientation_; // 期望姿态方向

  Vector6d      error; // 误差

  // TF变换:
  Matrix6d rotation_base_; // 基座到世界坐标系的旋转矩阵
  // 监听器
  tf::TransformListener listener_ft_; // 力矩传感器的TF监听器
  tf::TransformListener listener_control_; // 控制的TF监听器
  tf::TransformListener listener_arm_; // 机械臂的TF监听器

  // 状态标志
  bool ft_arm_ready_; // 力矩传感器就绪标志
  bool base_world_ready_; // 基座到世界坐标就绪标志
  bool world_arm_ready_; // 世界坐标到机械臂就绪标志

  double arm_max_vel_; // 机械臂最大速度
  double arm_max_acc_; // 机械臂最大加速度

  double force_x_pre, force_y_pre, force_z_pre; // 前一时刻的力

public:
  // 构造函数和析构函数
  Admittance(ros::NodeHandle &n, double frequency,
                      std::string topic_arm_state,
                      std::string topic_arm_command,
                      std::string topic_wrench_state,
                      std::vector<double> M,
                      std::vector<double> D,
                      std::vector<double> K,
                      std::vector<double> desired_pose,
                      std::string base_link,
                      std::string end_link,
                      double arm_max_vel,
                      double arm_max_acc
                       );
  ~Admittance(){}
  // 主运行函数
  void run();
private:
  // 控制函数
  void compute_admittance();
  // 回调函数
  void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);
  void state_wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg);

  // 向机器人发送命令的函数
  void send_commands_to_robot();

  // 等待变换的函数
  void wait_for_transformations();
  // 获取旋转矩阵的函数
  bool get_rotation_matrix(Matrix6d & rotation_matrix,
                           tf::TransformListener & listener,
                           std::string from_frame,  std::string to_frame);
private:
  std::string   base_link_; // 基座链接名称
  std::string   end_link_; // 末端链接名称
};

#endif // ADMITTANCE_H
