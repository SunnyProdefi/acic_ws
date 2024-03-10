/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:09:41 
 * @Last Modified by: MingshanHe
 * @Last Modified time: 2021-12-05 04:18:32
 * @Licence: MIT Licence
 */
#include <pluginlib/class_list_macros.h> // 导入pluginlib宏，用于插件的导出和注册
#include "cartesian_velocity_controller/kinematics_base.h" // 导入基础运动学类的定义
#include "cartesian_velocity_controller/cartesian_velocity_controller.h" // 导入笛卡尔速度控制器的定义
#include "kdl_conversions/kdl_msg.h" // 导入KDL消息转换函数

namespace cartesian_velocity_controller
{
/** \brief Initialize the kinematic chain for kinematics-based computation.
 *  \brief 初始化基于运动学的计算的运动链。
 */
bool Cartesian_Velocity_Controller::init(
    hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n) {

  // KDL 运动学基础初始化
  kinematics_base::Kinematics_Base::init(robot, n);

  // 使用给定的KDL链初始化逆运动学速度求解器，使用Givens旋转的伪逆方法
  ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_));
  // 使用给定的KDL链初始化正运动学速度求解器，递归方法
  fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain_));
  // 使用给定的KDL链初始化正运动学位置求解器，递归方法
  fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain_));

  // 获取发布周期
  if (!n.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set"); // 参数 'publish_rate' 未设置
      return false;
  }
  // 初始化实时发布器，用于发布末端执行器状态
  realtime_pub_.reset(new realtime_tools::RealtimePublisher
    <cartesian_state_msgs::PoseTwist>(n, "ee_state", 4));


  // 订阅命令主题
  sub_command_ = n.subscribe("command_cart_vel", 5,
    &Cartesian_Velocity_Controller::command_cart_vel, this,
    ros::TransportHints().reliable().tcpNoDelay());

  // 变量初始化
  this->joint_state_.resize(this->kdl_chain_.getNrOfJoints()); // 调整关节状态向量的大小以匹配KDL链中的关节数
  this->joint_effort_.resize(this->kdl_chain_.getNrOfJoints()); // 调整关节力矩向量的大小
  Jnt_Vel_Cmd_.resize(this->kdl_chain_.getNrOfJoints()); // 调整关节速度命令向量的大小
  End_Vel_Cmd_ = KDL::Twist::Zero(); // 初始化末端速度命令为零
  End_Pos_.p.Zero(); // 初始化末端位置向量为零
  End_Pos_.M.Identity(); // 初始化末端位置的方向为单位阵
  End_Vel_.p.Zero(); // 初始化末端速度向量为零
  End_Vel_.M.Identity(); // 初始化末端速度的方向为单位阵

  return true;
}

/** \brief This is called from within the realtime thread just before the
 * first call to \ref update
 * \brief 这个函数在实时线程中被调用，在第一次调用update函数之前。
 *
 * \param time 当前时间
 */
void Cartesian_Velocity_Controller::starting(const ros::Time& time){
  for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
    Jnt_Vel_Cmd_(i) = 0.0; // 初始化关节速度命令为0
    this->joint_state_.q(i)     = 0.0; // 初始化关节位置为0
    this->joint_state_.qdot(i)  = 0.0; // 初始化关节速度为0
    this->joint_effort_(i)    = 0.0; // 初始化关节力矩为0
  }
  End_Vel_Cmd_ = KDL::Twist::Zero(); // 初始化末端速度命令为零
  last_publish_time_ = time; // 更新最后一次发布时间
}

/*!
 * \brief Issues commands to the joint. Should be called at regular intervals
 * \brief 向关节发送命令。这个函数应该被定期调用。
 */
void Cartesian_Velocity_Controller::update(const ros::Time& time, const ros::Duration& period) {
  // 获取关节位置
  for(std::size_t i=0; i < this->joint_handles_.size(); i++)
  {
    // 更新关节状态：位置、速度、力矩
    this->joint_state_.q(i)         = this->joint_handles_[i].getPosition(); // 获取当前关节位置
    this->joint_state_.qdot(i)      = this->joint_handles_[i].getVelocity(); // 获取当前关节速度
    this->joint_effort_(i)        = this->joint_handles_[i].getEffort(); // 获取当前关节力矩
  }
  // 计算逆运动学速度求解
  ik_vel_solver_->CartToJnt(this->joint_state_.q, End_Vel_Cmd_, Jnt_Vel_Cmd_);
  // 发送速度命令到关节
  writeVelocityCommands(period);

  // 正向运动学计算
  fk_vel_solver_->JntToCart(this->joint_state_, End_Vel_); // 计算当前末端速度
  fk_pos_solver_->JntToCart(this->joint_state_.q, End_Pos_); // 计算当前末端位置

  // 限制发布率
  if (publish_rate_ > 0.0 && last_publish_time_
       + ros::Duration(1.0/publish_rate_) < time) {

    // 尝试发布
    if (realtime_pub_->trylock()) {
      // 更新发布时间
      last_publish_time_ = last_publish_time_
                           + ros::Duration(1.0/publish_rate_);
      // 填充消息
      realtime_pub_->msg_.header.stamp = time; // 设置时间戳
      tf::poseKDLToMsg(End_Pos_, realtime_pub_->msg_.pose); // 将KDL位姿转换为消息
      tf::twistKDLToMsg(End_Vel_.GetTwist(), realtime_pub_->msg_.twist); // 将KDL扭转转换为消息

      // 发布并解锁
      realtime_pub_->unlockAndPublish();
    }
  }
}

/*!
 * \brief Subscriber's callback: copies twist commands
 * \brief 订阅者的回调函数：复制旋转命令
 */
void Cartesian_Velocity_Controller::command_cart_vel(const geometry_msgs::TwistConstPtr &msg) {
    End_Vel_Cmd_.vel(0) = msg->linear.x; // 将消息中的线性速度x分量赋值给末端速度命令的线性部分
    End_Vel_Cmd_.vel(1) = msg->linear.y; // 将消息中的线性速度y分量赋值给末端速度命令的线性部分
    End_Vel_Cmd_.vel(2) = msg->linear.z; // 将消息中的线性速度z分量赋值给末端速度命令的线性部分
    End_Vel_Cmd_.rot(0) = msg->angular.x; // 将消息中的角速度x分量赋值给末端速度命令的旋转部分
    End_Vel_Cmd_.rot(1) = msg->angular.y; // 将消息中的角速度y分量赋值给末端速度命令的旋转部分
    End_Vel_Cmd_.rot(2) = msg->angular.z; // 将消息中的角速度z分量赋值给末端速度命令的旋转部分
}

/********************************************/
/**FUNCTIONS OF INSTANCES OF THE BASE CLASS**/
/********************************************/
// 基类实例的函数

/** \brief write the desired velocity command in the hardware interface input
 * for a PositionJointInterface
 * \brief 将期望的速度命令写入硬件接口输入，用于PositionJointInterface
 * \param period 更新周期的持续时间
 */
void Cartesian_Velocity_Controller::writeVelocityCommands(
                                    const ros::Duration& period) {
    for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
      // 为每个关节设置命令，根据当前周期计算新的期望位置
      this->joint_handles_[i].setCommand(this->joint_state_.q(i)
                                      + Jnt_Vel_Cmd_(i)*period.toSec());
    }
}

} // controller_interface 命名空间

// 使用PLUGINLIB_EXPORT_CLASS宏注册控制器，以便通过控制器管理器进行动态加载
PLUGINLIB_EXPORT_CLASS(cartesian_velocity_controller::Cartesian_Velocity_Controller,
                       controller_interface::ControllerBase)
