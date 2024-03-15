#include "impedance/Impedance.h"     // 包含阻抗控制器的头文件
#include "impedance/KDL_Base.h"      // 包含KDL基础类的头文件
#include "kdl_conversions/kdl_msg.h" // 包含KDL与ROS消息转换的头文件
#include <math.h>                    // 包含数学库

// 定义Impedance类的init函数，用于初始化
void Impedance::init(ros::NodeHandle &nh,
                     std::string topic_arm_state,
                     std::string topic_arm_command,
                     std::string topic_wrench_state,
                     std::vector<double> Ka,
                     std::vector<double> Kv,
                     std::vector<double> Kp,
                     std::vector<double> M,
                     std::vector<double> D,
                     std::vector<double> K,
                     std::vector<double> desired_pose)
{
    // 订阅者
    sub_arm_state_ = nh_.subscribe(topic_arm_state, 5,
                                   &Impedance::state_arm_callback, this, ros::TransportHints().reliable().tcpNoDelay()); // 订阅机械臂状态
    sub_wrench_state_ = nh_.subscribe(topic_wrench_state, 5,
                                      &Impedance::state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay()); // 订阅扭矩状态
    sub_posture_ = nh_.subscribe("/joint_torque_controller/command", 1,
                                 &Impedance::command, this, ros::TransportHints().reliable().tcpNoDelay()); // 订阅姿态
    // 发布者
    pub_arm_cmd_ = nh_.advertise<joint_effort_msg::JointEfforts>(topic_arm_command, 5); // 发布机械臂命令

    // KDL相关初始化
    kdl_base::KDL_Base::init(nh);  // 调用基类的init函数进行初始化
    Gravity = KDL::Vector::Zero(); // 重力向量初始化为0
    Gravity(2) = -9.81;            // 设置Z轴方向的重力加速度

    // 初始化KDL链的控制参数
    Kp_.resize(this->kdl_chain_.getNrOfJoints()); // 位置控制增益
    Kv_.resize(this->kdl_chain_.getNrOfJoints()); // 速度控制增益
    Ka_.resize(this->kdl_chain_.getNrOfJoints()); // 加速度控制增益
    M_.resize(this->kdl_chain_.getNrOfJoints());  // 惯性矩阵
    C_.resize(this->kdl_chain_.getNrOfJoints());  // 科里奥利力矩阵
    G_.resize(this->kdl_chain_.getNrOfJoints());  // 重力矩阵

    // 初始化KDL运动学求解器
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_)); // 逆速度求解器
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain_));   // 正向位置求解器
    fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain_));   // 正向速度求解器
    // KDL::ChainIkSolverVel_pinv_givens ik_vel_solver_ = KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_);
    // KDL::ChainFkSolverPos_recursive fk_pos_solver_ = KDL::ChainFkSolverPos_recursive(this->kdl_chain_);
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR(this->kdl_chain_, *(fk_pos_solver_.get()), *(ik_vel_solver_.get()))); // 逆位置求解器

    desired_pose_ = desired_pose;                                                                                 // 设置期望姿态
    Desired_Pos_ = KDL::Vector(desired_pose[0], desired_pose[1], desired_pose[2]);                                // 期望位置
    Desired_Ori_ = KDL::Rotation::Quaternion(desired_pose[3], desired_pose[4], desired_pose[5], desired_pose[6]); // 期望方向（四元数）
    Desired_Pose_ = KDL::Frame(Desired_Ori_, Desired_Pos_);                                                       // 期望姿态

    // 初始化KDL动力学求解器
    id_pos_solver_.reset(new KDL::ChainIdSolver_RNE(this->kdl_chain_, Gravity)); // 逆动力学求解器
    id_solver_.reset(new KDL::ChainDynParam(this->kdl_chain_, Gravity));         // 动力学参数求解器

    // 获取发布周期
    if (!nh.getParam("publish_rate", publish_rate_))
    {
        ROS_ERROR("Parameter 'publish_rate' not set"); // 如果没有设置publish_rate参数，则报错
    }

    // 变量初始化
    Jnt_Pos_Init_State.resize(this->kdl_chain_.getNrOfJoints()); // 关节初始位置
    Jnt_Desired_State.resize(this->kdl_chain_.getNrOfJoints());  // 关节期望状态
    CMD_State.resize(this->kdl_chain_.getNrOfJoints());          // 命令状态
    Current_State.resize(this->kdl_chain_.getNrOfJoints());      // 当前状态

    Jnt_Pos_State.resize(this->kdl_chain_.getNrOfJoints()); // 关节位置状态
    Jnt_Vel_State.resize(this->kdl_chain_.getNrOfJoints()); // 关节速度状态
    Jnt_Toq_State.resize(this->kdl_chain_.getNrOfJoints()); // 关节扭矩状态

    Jnt_Toq_Cmd_.resize(this->kdl_chain_.getNrOfJoints()); // 关节扭矩命令

    Ext_Wrenches.resize(kdl_chain_.getNrOfSegments());                            // 外部扭矩
    KDL::Wrench wrench = KDL::Wrench(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0)); // 初始化一个零扭矩
    Ext_Wrenches.back() = wrench;                                                 // 设置外部扭矩

    for (size_t i = 0; i < this->kdl_chain_.getNrOfJoints(); i++) // 遍历所有关节
    {
        Kp_(i) = Kp[i]; // 设置位置控制增益
        Kv_(i) = Kv[i]; // 设置速度控制增益
        Ka_(i) = Ka[i]; // 设置加速度控制增益
    }

    Impedance_M = M; // 质量
    Impedance_D = D; // 阻尼
    Impedance_K = K; // 刚度

    Recieved_Joint_State = false; // 初始化未接收关节状态
    Cmd_Flag_ = true;             // 命令标志设置为true
    Init_Flag_ = true;            // 初始化标志设置为true
    Step_ = 0;                    // 步骤计数器初始化

    wrench_z = 0; // Z轴扭矩初始化为0
    pos_z = 0;    // Z轴位置初始化为0
}
// 处理机械臂状态消息的回调函数
void Impedance::state_arm_callback(const joint_state_msg::JointState msg)
{
    for (size_t i = 0; i < this->kdl_chain_.getNrOfJoints(); i++) // 遍历所有关节
    {
        Jnt_Pos_State(i) = msg.position[i]; // 更新关节位置状态
        Jnt_Vel_State(i) = msg.velocity[i]; // 更新关节速度状态
        Jnt_Toq_State(i) = msg.effort[i];   // 更新关节扭矩状态
        Recieved_Joint_State = true;        // 标记已接收到关节状态
    }
    // 下面这行代码是被注释掉的，用于调试时打印关节位置状态，但在实际运行时通常不需要
    // std::cout<<Jnt_Pos_State(0)<<","<<Jnt_Pos_State(1)<<","<<Jnt_Pos_State(2)<<","
    // <<Jnt_Pos_State(3)<<","<<Jnt_Pos_State(4)<<","<<Jnt_Pos_State(5)<<std::endl;
    // std::cout<<"Recieved Joint State"<<std::endl;
}

// 处理扭矩状态消息的回调函数
void Impedance::state_wrench_callback(
    const geometry_msgs::WrenchConstPtr msg)
{
    KDL::Wrench wrench = KDL::Wrench(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0)); // 创建一个全为零的扭矩变量
    Ext_Wrenches.back() = wrench;                                                 // 将最后一个扭矩更新为零（这个步骤可能是为了清空之前的值）

    wrench_x = msg->force.x; // 更新X方向上的扭矩
    wrench_y = msg->force.y; // 更新Y方向上的扭矩
    wrench_z = msg->force.z; // 更新Z方向上的扭矩
}

void Impedance::compute_impedance(bool flag)
{
    // 如果flag为真，执行以下计算
    if (flag)
    {
        // 方法1
        if (Init_Flag_)
        {
            // 如果是初始状态，则将当前关节位置作为初始位置
            Jnt_Pos_Init_State = Jnt_Pos_State;
            // 将初始标志置为假，表示已经初始化
            Init_Flag_ = false;
        }

        // 如果Z轴的外力不为0
        if (wrench_z != 0)
        {
            KDL::Frame End_Pose;
            // 计算当前关节状态对应的末端位姿
            fk_pos_solver_->JntToCart(Jnt_Pos_State, End_Pose);

            KDL::JntArrayVel Jnt_Vel;
            KDL::FrameVel End_Pose_Vel;
            // 将当前关节位置和速度状态设置为关节速度数组
            Jnt_Vel = KDL::JntArrayVel(Jnt_Pos_State, Jnt_Vel_State);
            // 计算当前关节速度状态对应的末端速度
            fk_vel_solver_->JntToCart(Jnt_Vel, End_Pose_Vel);

            KDL::Vector pose_p, pose_vel_p;
            // 获取末端位姿的位置
            pose_p = End_Pose.p;
            // 获取末端速度的位置分量
            pose_vel_p = End_Pose_Vel.p.p;

            // double acc_x = (wrench_x - (Impedance_D[0]*pose_vel_p(0) + Impedance_K[0]*(pose_p(0)-desired_pose_[0])))/Impedance_M[0];
            // double acc_y = (wrench_y - (Impedance_D[1]*pose_vel_p(1) + Impedance_K[1]*(pose_p(1)-desired_pose_[1])))/Impedance_M[1];
            // 计算Z轴的加速度，考虑了质量、阻尼和弹性系数
            double acc_z = (wrench_z - (Impedance_D[2] * pose_vel_p(2) + Impedance_K[2] * (desired_pose_[2] - pose_p(2)))) / Impedance_M[2];

            ros::Rate loop_rate_(200);
            ros::Duration duration = loop_rate_.expectedCycleTime();
            // pos_x = pos_x + 0.01*(pose_vel_p(0) * duration.toSec() + 0.5 * acc_x * duration.toSec() * duration.toSec());
            // pos_y = pos_y + 0.01*(pose_vel_p(1) * duration.toSec() + 0.5 * acc_y * duration.toSec() * duration.toSec());
            // 根据加速度和速度更新Z轴的位置
            pos_z = 10 * (pose_vel_p(2) * duration.toSec() + 0.5 * acc_z * duration.toSec() * duration.toSec());
            // 设置期望位置
            Desired_Pos_ = KDL::Vector(desired_pose_[0] + pos_x, desired_pose_[1] + pos_y, desired_pose_[2] + pos_z);
            // 设置期望姿态
            Desired_Ori_ = KDL::Rotation::Quaternion(desired_pose_[3], desired_pose_[4], desired_pose_[5], desired_pose_[6]);
            // 组合期望的位置和姿态为期望的位姿
            Desired_Pose_ = KDL::Frame(Desired_Ori_, Desired_Pos_);
        }

        // 使用逆运动学求解器计算达到期望位姿的关节命令
        ik_pos_solver_->CartToJnt(Jnt_Pos_State, Desired_Pose_, CMD_State);

        // 使用双曲正切函数平滑达到期望的关节位置
        double lambda = 0.1;
        double th = tanh(M_PI - lambda * Step_);
        double ch = cosh(M_PI - lambda * Step_);
        double sh2 = 1.0 / (ch * ch);

        for (size_t i = 0; i < this->kdl_chain_.getNrOfJoints(); i++)
        {
            // 计算当前的关节状态
            Current_State(i) = CMD_State(i) - Jnt_Pos_Init_State(i);
            // 计算期望的关节位置
            Jnt_Desired_State.q(i) = Current_State(i) * 0.5 * (1.0 - th) + Jnt_Pos_Init_State(i);
            // 计算期望的关节速度
            Jnt_Desired_State.qdot(i) = Current_State(i) * 0.5 * lambda * sh2;
            // 计算期望的关节加速度
            Jnt_Desired_State.qdotdot(i) = Current_State(i) * lambda * lambda * sh2 * th;
        }
        // std::cout<<Current_State(0)<<","<<Current_State(1)<<","<<Current_State(2)<<","
        // <<Current_State(3)<<","<<Current_State(4)<<","<<Current_State(5)<<std::endl;
        // 如果达到命令的状态，则重置命令标志，准备下一次命令
        ++Step_;
        if (Jnt_Desired_State.q == CMD_State)
        {
            Cmd_Flag_ = false;                  // 重置命令标志
            Step_ = 0;                          // 重置步骤
            Jnt_Pos_Init_State = Jnt_Pos_State; // 更新初始状态
            // 输出姿态OK的信息
            // ROS_INFO("Posture OK");
        }

        // 计算惯性矩阵、科氏力矩阵和重力矩阵
        id_solver_->JntToMass(Jnt_Pos_State, M_);
        id_solver_->JntToCoriolis(Jnt_Pos_State, Jnt_Vel_State, C_);
        id_solver_->JntToGravity(Jnt_Pos_State, G_);

        // PID控制器
        KDL::JntArray pid_cmd_(this->kdl_chain_.getNrOfJoints());
        // 补偿科氏力和重力
        KDL::JntArray cg_cmd_(this->kdl_chain_.getNrOfJoints());

        for (size_t i = 0; i < this->kdl_chain_.getNrOfJoints(); i++)
        {
            // 控制律
            pid_cmd_(i) = Ka_(i) * Jnt_Desired_State.qdotdot(i) + Kv_(i) * (Jnt_Desired_State.qdot(i) - Jnt_Vel_State(i)) + Kp_(i) * (Jnt_Desired_State.q(i) - Jnt_Pos_State(i));
            cg_cmd_(i) = C_(i) + G_(i);
            // 计算关节扭矩命令
            // cg_cmd_(i) = C_(i)*Jnt_Desired_State.qdot(i) + G_(i);
            // Jnt_Toq_Cmd_(i) = M_(i)*Jnt_Desired_State.qdotdot(i)+C_(i)*Jnt_Desired_State.qdot(i)+G_(i);
        }
        Jnt_Toq_Cmd_.data = M_.data * pid_cmd_.data;
        KDL::Add(Jnt_Toq_Cmd_, cg_cmd_, Jnt_Toq_Cmd_);

        // 方法2，用于测试
        // 如果逆动力学求解器失败，则将所有扭矩设置为零
        // if(id_pos_solver_->CartToJnt(Jnt_Pos_State, Jnt_Vel_State, Jnt_Acc_Cmd_, Ext_Wrenches, Jnt_Toq_Cmd_)!=0)
        // {
        //     ROS_ERROR("Could not compute joint torques! Setting all torques to zero!");
        //     KDL::SetToZero(Jnt_Toq_Cmd_);
        // }
        // 发送命令到机器人
        send_commands_to_robot();

        // 重置接收到的关节状态标志
        Recieved_Joint_State = false;
    }
}

// 处理从外部接收到的命令
void Impedance::command(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    if (msg->data.size() == 0)
        ROS_INFO("Desired configuration must be of dimension %lu", this->kdl_chain_.getNrOfJoints()); // 如果接收到的数据为空，则提示所需数据的维度
    else if (msg->data.size() != this->kdl_chain_.getNrOfJoints())
    {
        ROS_ERROR("Posture message had the wrong size: %u", (unsigned int)msg->data.size()); // 如果数据大小不符合关节数量，则报错
        return;
    }
    else
    {
        for (unsigned int i = 0; i < this->kdl_chain_.getNrOfJoints(); i++)
            CMD_State(i) = msg->data[i];         // 将接收到的数据更新到CMD_State中，为每个关节设置新的命令
        std::cout << "Command SET" << std::endl; // 输出命令设置完成的信息
        Cmd_Flag_ = true;                        // 设置命令标志位为true
        Step_ = 0;                               // 重置步骤计数器，避免在更新时出现跳跃
    }
}

// 将计算出的关节扭矩命令发送到机器人
void Impedance::send_commands_to_robot()
{
    joint_effort_msg::JointEfforts msg; // 创建一个JointEfforts消息

    // 将计算出的扭矩命令赋值给消息
    msg.Joint1Effort = Jnt_Toq_Cmd_(0);
    msg.Joint2Effort = Jnt_Toq_Cmd_(1);
    msg.Joint3Effort = Jnt_Toq_Cmd_(2);
    msg.Joint4Effort = Jnt_Toq_Cmd_(3);
    msg.Joint5Effort = Jnt_Toq_Cmd_(4);
    msg.Joint6Effort = Jnt_Toq_Cmd_(5);
    // std::cout<<Jnt_Toq_Cmd_(0)<<","<<Jnt_Toq_Cmd_(1)<<","<<Jnt_Toq_Cmd_(2)<<","
    // <<Jnt_Toq_Cmd_(3)<<","<<Jnt_Toq_Cmd_(4)<<","<<Jnt_Toq_Cmd_(5)<<std::endl;
    pub_arm_cmd_.publish(msg); // 发布消息
}

// 运行阻抗控制循环
void Impedance::run()
{
    ROS_INFO("Running the impedance control loop ................."); // 输出正在运行阻抗控制循环的信息
    ros::Rate loop_rate_(200);                                        // 设置循环频率为200Hz

    while (nh_.ok())
    {
        compute_impedance(Recieved_Joint_State); // 计算阻抗控制

        ros::spinOnce(); // 处理一次回调函数

        loop_rate_.sleep(); // 等待直到达到循环频率设定的时间
    }
}