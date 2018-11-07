#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

if __name__ == '__main__':
    rospy.init_node('test_position_cmd')
    pub = rospy.Publisher('/turtlebot_arm/position_cmd', JointState, queue_size=10)
    while pub.get_num_connections() == 0:
        rospy.Rate(5).sleep()

    j = JointState()
    j.name = ['joint_1', 'joint_2', 'joint_3', 'gripper']
    j.position = [512, 128, 512, 512]
    j.header.stamp = rospy.Time().now()
    pub.publish(j)
    rospy.loginfo("Fin.")
