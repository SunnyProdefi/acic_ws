#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def set_initial_pose():
    rospy.init_node('ur5_joint_initializer', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # 等待连接
    while pub.get_num_connections() == 0:
        rospy.sleep(1)

    initial_state = JointState()
    initial_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    initial_state.position = [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]  # 小臂抬起90度
    initial_state.velocity = [0.0] * 6
    initial_state.effort = [0.0] * 6

    rospy.loginfo("Setting UR5 initial pose...")
    pub.publish(initial_state)
    rospy.sleep(1)  # 确保消息被发送

if __name__ == '__main__':
    try:
        set_initial_pose()
    except rospy.ROSInterruptException:
        pass
