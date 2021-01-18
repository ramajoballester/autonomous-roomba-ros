#!/usr/bin/env python

import rospy
import sys
import tf
from geometry_msgs.msg import PoseStamped

my_ns = "roomba"        # Default name

if __name__ == '__main__':
    if len(sys.argv) > 1:
        my_ns = sys.argv[1]
    rospy.init_node(my_ns + '_goal_publisher')
    if my_ns == "roomba":
        print("\nStarted process with default namespace: " + my_ns)
    else:
        print("\nStarted process with namespace: " + my_ns)


    pub = rospy.Publisher(my_ns + '/move_base_simple/goal', PoseStamped, queue_size=1)
    goal = PoseStamped()

    while not rospy.is_shutdown():
        x = float(input("X position: "))
        y = float(input("Y position: "))
        yaw = float(input("Yaw orientation: "))

        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]

        pub.publish(goal)
