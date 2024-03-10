/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:09:28 
 * @Last Modified by:   MingshanHe 
 * @Last Modified time: 2021-12-05 04:09:28 
 * @Licence: MIT Licence
 */
#ifndef CARTESIAN_VELOCITY_CONTROLLER_H
#define CARTESIAN_VELOCITY_CONTROLLER_H

// 引入所需的头文件
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <realtime_tools/realtime_publisher.h>
#include "kinematics_base.h" // 包括运动学基础类的定义

namespace cartesian_velocity_controller
{

/** \brief 这个类实现了一个ROS控制的笛卡尔速度控制器。它的基类实现了控制器的核心。
 */
class Cartesian_Velocity_Controller: public kinematics_base::Kinematics_Base
{
public:
  Cartesian_Velocity_Controller() {}
  ~Cartesian_Velocity_Controller() {}

  /** \brief init函数用于从非实时线程初始化控制器，并传入硬件接口的指针。
   * \param robot 该控制器使用的特定硬件接口。
   * \param n 一个NodeHandle，控制器应从其命名空间读取配置，并在其中设置其ROS接口。
   * \returns 如果初始化成功并且控制器准备好启动，则返回True。
   */
  bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

  /** \brief 在第一次调用update之前，从实时线程调用这个函数。
   * \param time 当前时间
   */
  void starting(const ros::Time& time);

  /*!
   * \brief 向关节发送命令。定期调用此函数。
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /*!
   * \brief 订阅者的回调函数。
   */
  void command_cart_vel(const geometry_msgs::TwistConstPtr &msg);
private:
  /** \brief 将当前命令写入硬件接口。
   */
  void writeVelocityCommands(const ros::Duration& period);

protected:
  ros::Subscriber                 sub_command_; // 接收外部命令的接口

  ros::Time                       last_publish_time_; // 上次发布时间
  double                          publish_rate_; // 发布率

  KDL::JntArray                   Jnt_Vel_Cmd_;      // 期望的关节速度
  KDL::Twist                      End_Vel_Cmd_;      // 期望的末端执行器速度
  KDL::FrameVel                   End_Vel_; // 末端速度
  KDL::Frame                      End_Pos_; // 末端位置
  cartesian_state_msgs::PoseTwist msg_state_; // 状态消息


  boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_; // 正向速度解算器
  boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_; // 正向位置解算器
  boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_; // 逆向速度解算器

  boost::shared_ptr<realtime_tools::RealtimePublisher<
     cartesian_state_msgs::PoseTwist> > realtime_pub_; // 实时发布器
};
} // namespace controller_interface


#endif

