#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def set_initial_pose():
    rospy.init_node('ur5_joint_initializer', anonymous=True)
    # 假设控制器的命令话题是'/pos_joint_traj_controller/command'
    pub = rospy.Publisher('/pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

    # 等待连接
    while pub.get_num_connections() == 0:
        rospy.sleep(1)

    # 构造JointTrajectory消息
    trajectory = JointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    # 创建一个轨迹点
    point = JointTrajectoryPoint()
    point.positions = [0, -1.57, 1.57, 0.0, 0.0, 0.0]  # 目标位置
    point.time_from_start = rospy.Duration(1)  # 从开始到达目标位置的时间
    trajectory.points.append(point)

    rospy.loginfo("Setting UR5 initial pose using JointTrajectoryController...")
    pub.publish(trajectory)
    rospy.sleep(1)  # 确保消息被发送

if __name__ == '__main__':
    try:
        set_initial_pose()
    except rospy.ROSInterruptException:
        pass
