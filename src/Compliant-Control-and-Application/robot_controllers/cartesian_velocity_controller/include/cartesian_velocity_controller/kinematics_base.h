/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:09:34 
 * @Last Modified by:   MingshanHe 
 * @Last Modified time: 2021-12-05 04:09:34 
 * @Licence: MIT Licence
 */
#ifndef CARTESIAN_VELOCITY_CONTROLLER_KINEMATICS_BASE_H
#define CARTESIAN_VELOCITY_CONTROLLER_KINEMATICS_BASE_H

// 引入必要的头文件
#include <urdf/model.h> // 用于处理URDF模型
#include <controller_interface/controller.h> // 控制器接口基类
#include <hardware_interface/joint_command_interface.h> // 硬件接口，用于发送关节指令

#include <ros/node_handle.h> // ROS节点句柄，用于与ROS通信
#include <ros/ros.h> // ROS的主要功能

#include <kdl/tree.hpp> // KDL树结构
#include <kdl/kdl.hpp> // KDL的核心功能
#include <kdl/chain.hpp> // KDL链
#include <kdl/chainfksolver.hpp> // KDL正向运动学求解器
#include <kdl/frames.hpp> // KDL框架
#include <kdl/chaindynparam.hpp> // 用于计算重力向量的KDL链动态参数
#include <kdl/chainjnttojacsolver.hpp> // KDL链到雅可比矩阵求解器
#include <kdl/chainfksolverpos_recursive.hpp> // KDL链正向运动学位置求解器（递归）
#include <kdl/chainfksolvervel_recursive.hpp> // KDL链正向运动学速度求解器（递归）
#include <kdl_parser/kdl_parser.hpp> // KDL解析器，用于从URDF模型中生成KDL数据结构

#include <vector> // 向量库

namespace kinematics_base
{
// 定义Kinematics_Base类，继承自controller_interface::Controller
class Kinematics_Base: public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  Kinematics_Base() {} // 构造函数
  ~Kinematics_Base() {} // 析构函数

  // 初始化函数，用于初始化控制器
  bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

protected:
  ros::NodeHandle   nh_; // ROS节点句柄
  KDL::Chain        kdl_chain_; // KDL链
  KDL::JntArrayVel  joint_state_; // 关节状态（位置和速度）
  KDL::JntArray     joint_effort_; // 关节力矩

  // 定义一个结构体，用于存储关节限制信息
  struct limits_
  {
    KDL::JntArray min; // 最小值
    KDL::JntArray max; // 最大值
    KDL::JntArray center; // 中心值
  } joint_limits_;

  std::vector<hardware_interface::JointHandle> joint_handles_; // 关节句柄的向量
};

// Kinematics_Base类的初始化函数
bool Kinematics_Base::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
{
  nh_ = n; // 将节点句柄赋值给成员变量

  // 从参数服务器获取URDF描述以及根节点和末端节点的名称
  std::string robot_description, root_name, tip_name;

  std::string name_space = nh_.getNamespace(); // 获取节点的命名空间
  std::cout<< "--------------------> name_space:  " << name_space << std::endl; // 打印命名空间

  // 检查参数服务器上是否存在robot_description参数
  if (!ros::param::search(name_space,"robot_description", robot_description))
  {
    ROS_ERROR_STREAM("Kinematics_Base: No robot description (URDF)"
                    "found on parameter server (" << nh_.getNamespace() <<
                    "/robot_description)"); // 若未找到，则报错
    return false;
  }

  // 获取根节点名称
  if (!nh_.getParam( name_space + "/root_name", root_name))
  {
    ROS_ERROR_STREAM("Kinematics_Base: No root name found on "
                    "parameter server ("<<nh_.getNamespace()<<"/root_name)");
    return false; // 若未找到，则报错
  }

  // 获取末端节点名称
  if (!nh_.getParam(name_space + "/tip_name", tip_name))
  {
    ROS_ERROR_STREAM("Kinematics_Base: No tip name found on "
                    "parameter server ("<<nh_.getNamespace()<<"/tip_name)");
    return false; // 若未找到，则报错
  }

  // 从参数服务器获取URDF XML字符串
  std::string xml_string;
  if (nh_.hasParam(robot_description))
    nh_.getParam(robot_description.c_str(), xml_string);
  else
  {
    ROS_ERROR("Parameter %s not set, shutting down node...",
              robot_description.c_str()); // 若未设置参数，则关闭节点
    nh_.shutdown();
    return false;
  }

  if (xml_string.size() == 0)
  {
    ROS_ERROR("Unable to load robot model from parameter %s",
              robot_description.c_str()); // 若URDF字符串为空，则报错
    nh_.shutdown();
    return false;
  }

  // 将URDF XML字符串初始化为urdf::Model对象
  urdf::Model model;
  if (!model.initString(xml_string))
  {
    ROS_ERROR("Failed to parse urdf file"); // 解析失败则报错
    nh_.shutdown();
    return false;
  }
  ROS_INFO("Successfully parsed urdf file"); // 解析成功

  // 从urdf::Model构建KDL树
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Failed to construct kdl tree"); // 构建KDL树失败则报错
    nh_.shutdown();
    return false;
  }

  // 从KDL树获取KDL链
  if(!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
    ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
    ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
    ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
    ROS_ERROR_STREAM("  The segments are:");

    KDL::SegmentMap segment_map = kdl_tree.getSegments();
    KDL::SegmentMap::iterator it;

    for( it=segment_map.begin(); it != segment_map.end(); it++ )
      ROS_ERROR_STREAM( "    "<<(*it).first);

    return false; // 获取KDL链失败则报错
  }

  // 打印根节点和末端节点的名称以及链中的关节和段数
  ROS_INFO("tip_name:  %s",tip_name.c_str());
  ROS_INFO("root_name: %s",root_name.c_str());
  ROS_INFO("Number of segments: %d", kdl_chain_.getNrOfSegments());
  ROS_INFO("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
  for(std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); i++){
    ROS_INFO_STREAM("segment("<<i<<"): " << kdl_chain_.getSegment(i).getName());
  }

  // 从urdf模型中解析关节极限，并存储于kdl链中
  std::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);
  std::shared_ptr<const urdf::Joint> joint_;
  joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
  joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
  joint_limits_.center.resize(kdl_chain_.getNrOfJoints());
  int index;

  for (std::size_t i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
  {
    joint_ = model.getJoint(link_->parent_joint->name);
    ROS_INFO("Getting limits for joint: %s", joint_->name.c_str());
    index = kdl_chain_.getNrOfJoints() - i - 1;

    if(joint_->limits){
      joint_limits_.min(index) = joint_->limits->lower;
      joint_limits_.max(index) = joint_->limits->upper;
      joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;
    }else{
      joint_limits_.min(index) = 0;
      joint_limits_.max(index) = 0;
      joint_limits_.center(index) = 0;
      ROS_INFO("joint_->limits is NULL %s",joint_->name.c_str());
    }

    link_ = model.getLink(link_->getParent()->name);
  }

  ROS_INFO("Getting joint handles");
  // 获取链中所有关节的句柄
  int count=0;
  for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
  {

    ROS_INFO("%s type: %s", it->getJoint().getName().c_str(),
            it->getJoint().getTypeName().c_str() );
    if(it->getJoint().getTypeName() != "None" && count < 7) {
      joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
    }
    count++;
  }

  ROS_INFO("Number of joints in handle = %lu", joint_handles_.size() );
  ROS_INFO_STREAM("kdl_chain.getNrOfJoints: " << kdl_chain_.getNrOfJoints());

  ROS_INFO("Finished Kinematic Base init");

  return true;
}
}
#endif // CARTESIAN_VELOCITY_CONTROLLER_KINEMATICS_BASE_H

