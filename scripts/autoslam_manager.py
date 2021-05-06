#!/usr/bin/env python

import rospy
import sys
import time
import math
import rosnode
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist

class Manager():
    def __init__(self):
        self.n_vel_count = 0
        self.state = -1
        self.vel = 0
        self.init_time = 0
        self.goal = MoveBaseActionGoal()

        rospy.Subscriber('move_base/goal', MoveBaseActionGoal, self.goal_callback)
        rospy.Subscriber('move_base/status', GoalStatusArray, self.state_callback)
        rospy.Subscriber('cmd_vel', Twist, self.vel_callback)

    def state_callback(self, msg):          # Update robot state
        if len(msg.status_list) == 0:
            self.state = -1
        else:
            self.state = msg.status_list[-1].status

        # print('State')
        # print(self.state)

    def goal_callback(self, msg):
        self.goal = msg
        # print('Goal')
        # print(msg)

    def vel_callback(self, msg):
        if not self.n_vel_count:
            self.init_time = rospy.get_time()
        self.n_vel_count += 1
        self.vel = (self.vel*self.n_vel_count+msg.linear.x)/(self.n_vel_count+1)
        # print('Velocity')
        # print(self.vel)


if __name__ == '__main__':
    rospy.init_node('autoslam_manager')       # Init node bidder (under namespace)
    rate = rospy.Rate(1.0)

    manager = Manager()

    goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)

    x_init = rospy.get_param('x_init')
    y_init = rospy.get_param('y_init')

    done = False

    while not rospy.is_shutdown():

        if manager.state == 2:
            time.sleep(2.0)
        if manager.state == 2:
            if not done:
                print('Enter final step')

                manager.goal.header.stamp = rospy.Time.now()
                # manager.goal.goal_id.stamp = manager.goal.header.stamp
                manager.goal.goal_id.stamp.secs = 0
                manager.goal.goal_id.id = ""
                manager.goal.goal.target_pose.header.stamp = manager.goal.header.stamp

                manager.goal.goal.target_pose.pose.position.x = x_init
                manager.goal.goal.target_pose.pose.position.y = y_init
                # manager.goal.header.seq = 20

                goal_pub.publish(manager.goal)
                done = True
                print(' ')
                print(' ')
                print('Last goal published')
                print(' ')
                print(' ')

                time.sleep(1.0)

        elif manager.state == 3:
            time_elapsed = rospy.get_time()-manager.init_time
            print(' ')
            print(' ')
            print('Travelled distance:')
            print(manager.vel*time_elapsed)
            print('Time spent:')
            print(time_elapsed)
            print('Average velocity')
            print(manager.vel)
            print(' ')
            print(' ')

            sys.exit(0)


        rate.sleep()
