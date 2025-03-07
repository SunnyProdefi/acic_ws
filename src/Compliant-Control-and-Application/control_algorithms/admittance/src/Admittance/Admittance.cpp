/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:08:47 
 * @Last Modified by:   MingshanHe 
 * @Last Modified time: 2021-12-05 04:08:47 
 * @Licence: MIT Licence
 */
// 包含Admittance类的定义
#include <admittance/Admittance.h>

// Admittance类的构造函数
Admittance::Admittance(ros::NodeHandle &n,
    double frequency,
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
    double arm_max_acc) :
  nh_(n), loop_rate_(frequency),
  M_(M.data()), D_(D.data()),K_(K.data()),desired_pose_(desired_pose.data()),
  arm_max_vel_(arm_max_vel), arm_max_acc_(arm_max_acc),
  base_link_(base_link), end_link_(end_link){
  // 初始化订阅器和发布器
  //* 订阅器
  sub_arm_state_ = nh_.subscribe(topic_arm_state, 5, 
      &Admittance::state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
  sub_wrench_state_ = nh_.subscribe(topic_wrench_state, 5,
      &Admittance::state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  //* 发布器
  pub_arm_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 5);

  // 初始化类变量
  arm_position_.setZero();
  arm_twist_.setZero();
  wrench_external_.setZero();
  desired_pose_position_ << desired_pose_.topRows(3);
  desired_pose_orientation_.coeffs() << desired_pose_.bottomRows(4)/desired_pose_.bottomRows(4).norm();

  // 等待接收到机械臂的状态信息
  while (nh_.ok() && !arm_position_(0)) {
    ROS_WARN_THROTTLE(1, "Waiting for the state of the arm...");
    ros::spinOnce();
    loop_rate_.sleep();
  }

  // 初始化积分器
  arm_desired_twist_adm_.setZero();

  // 初始化传感器和转换矩阵是否准备好的标志
  ft_arm_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;

  // 初始化预先记录的力
  force_x_pre = 0;
  force_y_pre = 0;
  force_z_pre = 0;
  // 等待所有转换矩阵准备好
  wait_for_transformations();
}

// 等待转换矩阵的初始化
void Admittance::wait_for_transformations() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  // 确保在回调函数中启用所有转换前，所有TF转换已存在
  base_world_ready_ = true;
  world_arm_ready_ = true;
  // 循环直到从base_link到end_link的转换矩阵准备好
  while (!get_rotation_matrix(rot_matrix, listener, base_link_, end_link_)) {sleep(1);}
  ft_arm_ready_ = true;
  ROS_INFO("The Force/Torque sensor is ready to use.");
}

// 运行顺应性控制循环
void Admittance::run() {
  ROS_INFO("Running the admittance control loop .................");

  while (nh_.ok()) {
    // 计算顺应性控制
    compute_admittance();
    // 将命令发送给机器人
    send_commands_to_robot();
    // 处理ROS消息队列中的消息，并等待下一个循环
    ros::spinOnce();
    loop_rate_.sleep();
  }
}

void Admittance::compute_admittance() {
  error.topRows(3) = arm_position_ - desired_pose_position_;
  if(desired_pose_orientation_.coeffs().dot(arm_orientation_.coeffs()) < 0.0)
  {
    arm_orientation_.coeffs() << -arm_orientation_.coeffs();
  }
  Eigen::Quaterniond quat_rot_err (arm_orientation_ * desired_pose_orientation_.inverse());
  if(quat_rot_err.coeffs().norm() > 1e-3)
  {
    quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();
  Vector6d coupling_wrench_arm;
  coupling_wrench_arm=  D_ * (arm_desired_twist_adm_) + K_*error;
  arm_desired_accelaration = M_.inverse() * ( - coupling_wrench_arm  + wrench_external_);
  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();
  if (a_acc_norm > arm_max_acc_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm acceleration!"
                             << " norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }
  ros::Duration duration = loop_rate_.expectedCycleTime();
  arm_desired_twist_adm_ += arm_desired_accelaration * duration.toSec();
}

//!-                     回调函数                        -!//

// 机械臂状态的回调函数
void Admittance::state_arm_callback(
  const cartesian_state_msgs::PoseTwistConstPtr msg) {
  // 更新机械臂的位置
  arm_position_ <<  msg->pose.position.x,
                    msg->pose.position.y,
                    msg->pose.position.z;
  // 更新机械臂的方向
  arm_orientation_.coeffs() <<  msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z,
                                msg->pose.orientation.w;
  // 更新机械臂的扭力
  arm_twist_ << msg->twist.linear.x,
                msg->twist.linear.y,
                msg->twist.linear.z,
                msg->twist.angular.x,
                msg->twist.angular.y,
                msg->twist.angular.z;
}

// 外力状态的回调函数
void Admittance::state_wrench_callback(
  const geometry_msgs::WrenchStampedConstPtr msg) {
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;
  // 如果力矩传感器准备就绪
  if (ft_arm_ready_) {
    // 从消息中提取力矩
    wrench_ft_frame <<  msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z,0,0,0;

    // 下面的代码段是一个低通滤波器的例子，但在这里被注释掉了
    get_rotation_matrix(rotation_ft_base, listener_ft_, base_link_, end_link_);
    //
    // 将测得的力矩值转换到基座坐标系下
    wrench_external_ <<  rotation_ft_base * wrench_ft_frame;
  }
}

//!-               向机器人发送命令                    -!//
void Admittance::send_commands_to_robot() {
  // 创建一个Twist消息，用于发送速度命令
  geometry_msgs::Twist arm_twist_cmd;
  // 设置线速度和角速度，这里乘以0.3是为了降低速度，可能是为了安全或稳定性考虑
  arm_twist_cmd.linear.x  = arm_desired_twist_adm_(0)*0.3;
  arm_twist_cmd.linear.y  = arm_desired_twist_adm_(1)*0.3;
  arm_twist_cmd.linear.z  = arm_desired_twist_adm_(2)*0.3;
  arm_twist_cmd.angular.x = arm_desired_twist_adm_(3)*0.3;
  arm_twist_cmd.angular.y = arm_desired_twist_adm_(4)*0.3;
  arm_twist_cmd.angular.z = arm_desired_twist_adm_(5)*0.3;

  // 在终端中打印每次发送的速度值
  ROS_INFO_STREAM("Sending velocity command - "
                  << "Linear: [" << arm_twist_cmd.linear.x << ", "
                  << arm_twist_cmd.linear.y << ", "
                  << arm_twist_cmd.linear.z << "] "
                  << "Angular: [" << arm_twist_cmd.angular.x << ", "
                  << arm_twist_cmd.angular.y << ", "
                  << arm_twist_cmd.angular.z << "]");

  // 发布速度命令
  pub_arm_cmd_.publish(arm_twist_cmd);
}

//!-                    实用功能                        -!//
// 获取旋转矩阵的函数
bool Admittance::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try {
    // 试图从监听器获取从一个坐标帧到另一个坐标帧的变换
    listener.lookupTransform(from_frame, to_frame,
                            ros::Time(0), transform);
    // 将TF转换的旋转部分转换为Eigen矩阵
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    // 初始化旋转矩阵为零矩阵，然后填充对应的3x3块
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException& ex) {
    // 如果出现异常（比如变换不可用），则将旋转矩阵设置为零并返回false
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }
  return true;
}