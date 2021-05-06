#!/usr/bin/env python

import rospy
import sys
import time
import math
import rosnode
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from actionlib_msgs.msg import GoalStatusArray
from robotics.msg import Task, Bid, AuctionResult
from std_msgs.msg import Int16

wait_time_check_state = 2.0 # Time to wait before changing navigation state

class Robot():
    def __init__(self, robot_ns):
        self.robot_ns = robot_ns # namespace
        self.pose = Pose()
        self.velocity = 0.4 # max velocity (overridden by planner ros params)
        self.state = -1 # not active
        self.ready = 0 # not ready

        # Published topics
        self.bid_pub = rospy.Publisher('bid', Bid, queue_size=1)
        self.move_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.ready_pub = rospy.Publisher('bidder_ready', Int16, queue_size=1)

        # Subscribed topics
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/auction', Task, self.task_callback)
        rospy.Subscriber('/auction_result', AuctionResult, self.result_callback)
        rospy.Subscriber('move_base/status', GoalStatusArray, self.state_callback)


    def pose_callback(self, msg):   # Update robot pose
        self.pose = msg.pose.pose

    def task_callback(self, msg):   # Distance-bid calculation if ready
        if self.ready:
            distance = (math.sqrt((msg.x2-msg.x1)**2+(msg.y2-msg.y1)**2) +
                        math.sqrt((msg.x1-self.pose.position.x)**2+(msg.y1-self.pose.position.y)**2))

            self.bid_pub.publish(distance/self.velocity)

    def result_callback(self, msg):
        if msg.robot_id == self.robot_ns: # If robot is choosen by auctioneer:
            self.ready = 0
            self.ready_pub.publish(0)       # Temporarily not ready for future bids
            goal = self.calculate_goal(msg.task.x1, msg.task.y1)
            self.move_pub.publish(goal)     # Publish initial goal to navigation planner
            time.sleep(wait_time_check_state)
            while self.state != 3:  # While navigating, sleep auction process
                time.sleep(0.2)

            goal = self.calculate_goal(msg.task.x2, msg.task.y2)
            self.move_pub.publish(goal)     # Publish end goal
            time.sleep(wait_time_check_state)
            while self.state != 3:
                time.sleep(0.2)

            self.ready = 1
            self.ready_pub.publish(1)       # Ready again for next auction

    def state_callback(self, msg):          # Update robot state
        if len(msg.status_list) == 0:
            self.state = -1
        else:
            self.state = msg.status_list[-1].status

    def calculate_goal(self, x, y):         # Calculate goal for navigation planner
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0
        goal.pose.orientation.x = 0
        goal.pose.orientation.y = 0
        goal.pose.orientation.z = 0
        goal.pose.orientation.w = 1
        return goal



if __name__ == '__main__':
    rospy.init_node('bidder')       # Init node bidder (under namespace)
    rate = rospy.Rate(2.0)

    robot_ns = sys.argv[1]          # namespace
    robot = Robot(robot_ns)
    started = False

    while not rospy.is_shutdown():
        if not started:
            nodes = rosnode.get_node_names()

            if '/auctioneer' in nodes:      # If auctioneer ready, start
                started = True
                robot.velocity = rospy.get_param('move_base/TebLocalPlannerROS/max_vel_x')
                robot.ready_pub.publish(1)
                robot.ready = 1

        if robot.ready:             # Everything correct, robot is up and ready
            robot.ready_pub.publish(1)
        rate.sleep()
