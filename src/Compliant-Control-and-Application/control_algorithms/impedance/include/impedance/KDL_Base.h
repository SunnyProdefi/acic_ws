#ifndef KDL_BASE_H // 如果还未定义KDL_BASE_H
#define KDL_BASE_H // 则定义KDL_BASE_H

#include <urdf/model.h>                                 // 包含URDF模型的头文件
#include <controller_interface/controller.h>            // 包含控制器接口的头文件
#include <hardware_interface/joint_command_interface.h> // 包含关节命令接口的头文件

#include <ros/node_handle.h> // 包含ROS节点句柄的头文件
#include <ros/ros.h>         // 包含ROS的主要头文件

#include <kdl/tree.hpp>                       // 包含KDL树结构的头文件
#include <kdl/kdl.hpp>                        // 包含KDL的主要头文件
#include <kdl/chain.hpp>                      // 包含KDL链的头文件
#include <kdl/chainfksolver.hpp>              // 包含KDL正向运动学求解器的头文件
#include <kdl/frames.hpp>                     // 包含KDL框架的头文件
#include <kdl/chaindynparam.hpp>              // 包含计算重力向量的KDL链动态参数的头文件
#include <kdl/chainjnttojacsolver.hpp>        // 包含KDL链到雅可比求解器的头文件
#include <kdl/chainfksolverpos_recursive.hpp> // 包含KDL链正向运动学位置求解器的递归实现的头文件
#include <kdl/chainfksolvervel_recursive.hpp> // 包含KDL链正向运动学速度求解器的递归实现的头文件
#include <kdl_parser/kdl_parser.hpp>          // 包含KDL解析器的头文件

#include <vector> // 包含标准模板库中的向量头文件

// 定义kdl_base命名空间
namespace kdl_base
{
  // KDL_Base类定义
  class KDL_Base
  {
  public:
    KDL_Base() {}  // 构造函数
    ~KDL_Base() {} // 析构函数

    // 初始化函数
    bool init(ros::NodeHandle &n);

  protected:
    ros::NodeHandle nh_;           // ROS节点句柄
    KDL::Chain kdl_chain_;         // KDL链
    KDL::JntArrayVel joint_state_; // 关节状态（速度和位置）
    KDL::JntArray joint_effort_;   // 关节努力（力或扭矩）

    // 关节限制结构体
    struct limits_
    {
      KDL::JntArray min;    // 最小限制
      KDL::JntArray max;    // 最大限制
      KDL::JntArray center; // 中心位置
    } joint_limits_;        // 关节限制
  };

bool KDL_Base::init(ros::NodeHandle &n)  // 定义KDL_Base类的初始化函数
{
    nh_ = n;  // 将传入的ROS节点句柄赋值给类成员变量

    // 从参数服务器获取URDF和根、末端的名称
    std::string robot_description, root_name, tip_name;

    std::string name_space = nh_.getNamespace();  // 获取节点的命名空间
    std::cout << "--------------------> name_space:  " << name_space << std::endl;  // 打印命名空间

    // 搜索robot_description参数
    if (!ros::param::search(name_space, "robot_description", robot_description))
    {
      ROS_ERROR_STREAM("KDL_Base: No robot description (URDF)"
                       "found on parameter server ("
                       << nh_.getNamespace() << "/robot_description)");  // 若未找到，则报错
      return false;  // 返回false，表示初始化失败
    }

    // 获取根名称参数
    if (!nh_.getParam(name_space + "/root_name", root_name))
    {
      ROS_ERROR_STREAM("KDL_Base: No root name found on "
                       "parameter server ("
                       << nh_.getNamespace() << "/root_name)");  // 若未找到，则报错
      return false;  // 返回false，表示初始化失败
    }

    // 获取末端名称参数
    if (!nh_.getParam(name_space + "/tip_name", tip_name))
    {
      ROS_ERROR_STREAM("KDL_Base: No tip name found on "
                       "parameter server ("
                       << nh_.getNamespace() << "/tip_name)");  // 若未找到，则报错
      return false;  // 返回false，表示初始化失败
    }

    // 从xml字符串构造URDF模型
    std::string xml_string;

    if (nh_.hasParam(robot_description))
      nh_.getParam(robot_description.c_str(), xml_string);  // 如果参数存在，获取参数值
    else
    {
      ROS_ERROR("Parameter %s not set, shutting down node...",
                robot_description.c_str());  // 若参数未设置，则报错并关闭节点
      nh_.shutdown();
      return false;  // 返回false，表示初始化失败
    }

    if (xml_string.size() == 0)
    {
      ROS_ERROR("Unable to load robot model from parameter %s",
                robot_description.c_str());  // 若参数值为空，则报错
      nh_.shutdown();
      return false;  // 返回false，表示初始化失败
    }

    // 从robot_description中获取urdf模型
    urdf::Model model;
    if (!model.initString(xml_string))
    {
      ROS_ERROR("Failed to parse urdf file");  // 若解析失败，则报错
      nh_.shutdown();
      return false;  // 返回false，表示初始化失败
    }
    ROS_INFO("Successfully parsed urdf file");  // 解析成功

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
      ROS_ERROR("Failed to construct kdl tree");  // 若构造KDL树失败，则报错
      nh_.shutdown();
      return false;  // 返回false，表示初始化失败
    }

    // 填充KDL链
    if (!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
    {
      ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");  // 若获取链失败，则报错
      ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
      ROS_ERROR_STREAM("  Tree has " << kdl_tree.getNrOfJoints() << " joints");
      ROS_ERROR_STREAM("  Tree has " << kdl_tree.getNrOfSegments() << " segments");
      ROS_ERROR_STREAM("  The segments are:");

      KDL::SegmentMap segment_map = kdl_tree.getSegments();
      KDL::SegmentMap::iterator it;

      for (it = segment_map.begin(); it != segment_map.end(); it++)
        ROS_ERROR_STREAM("    " << (*it).first);  // 打印所有段的名称

      return false;  // 返回false，表示初始化失败
    }

    // 打印信息
    ROS_INFO("tip_name:  %s", tip_name.c_str());  // 打印末端名称
    ROS_INFO("root_name: %s", root_name.c_str());  // 打印根名称
    ROS_INFO("Number of segments: %d", kdl_chain_.getNrOfSegments());  // 打印链中段的数量
    ROS_INFO("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());  // 打印链中关节的数量
    for (std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); i++)
    {
      ROS_INFO_STREAM("segment(" << i << "): " << kdl_chain_.getSegment(i).getName());  // 遍历打印每个段的名称
    }

    // 从urdf模型中解析kdl链的关节限制
    std::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);  // 获取末端连杆的共享指针
    std::shared_ptr<const urdf::Joint> joint_;  // 定义一个用于存储当前关节的共享指针
    joint_limits_.min.resize(kdl_chain_.getNrOfJoints());  // 为最小限制分配空间
    joint_limits_.max.resize(kdl_chain_.getNrOfJoints());  // 为最大限制分配空间
    joint_limits_.center.resize(kdl_chain_.getNrOfJoints());  // 为中心位置分配空间
    int index;  // 用于存储当前关节在数组中的索引

    for (std::size_t i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)  // 遍历链中的所有关节
    {
      joint_ = model.getJoint(link_->parent_joint->name);  // 获取当前关节的信息
      ROS_INFO("Getting limits for joint: %s", joint_->name.c_str());  // 打印当前正在获取限制信息的关节名称
      index = kdl_chain_.getNrOfJoints() - i - 1;  // 计算当前关节在数组中的索引

      if (joint_->limits)  // 如果关节有限制信息
      {
        joint_limits_.min(index) = joint_->limits->lower;  // 设置最小限制
        joint_limits_.max(index) = joint_->limits->upper;  // 设置最大限制
        joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index)) / 2;  // 计算中心位置
      }
      else  // 如果关节没有限制信息
      {
        joint_limits_.min(index) = 0;  // 将最小限制设置为0
        joint_limits_.max(index) = 0;  // 将最大限制设置为0
        joint_limits_.center(index) = 0;  // 将中心位置设置为0
        ROS_INFO("joint_->limits is NULL %s", joint_->name.c_str());  // 打印关节没有限制信息的提示
      }

      link_ = model.getLink(link_->getParent()->name);  // 获取当前连杆的父连杆，用于下一次迭代
    }

    ROS_INFO("Finished Kinematic Base init");  // 打印初始化完成的信息

    return true;  // 返回true，表示初始化成功
  }
}

#endif // KDL_BASE_H
