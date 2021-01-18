#!/usr/bin/env python3

import rospy
import pygame
import sys
from geometry_msgs.msg import Twist

pygame.init()
pygame.display.set_mode((200, 200))
print(sys.version)

vel_max = 1.0
w_max = 1.0
vel = 0
w = 0

if __name__ == '__main__':
    rospy.init_node('keyboard')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(100.0)

    rospy.Time()

    cmd = Twist()
    cmd.linear.x = 0
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = 0

    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    cmd.linear.x = cmd.linear.x + vel_max
                if event.key == pygame.K_DOWN:
                    cmd.linear.x = cmd.linear.x - vel_max
                if event.key == pygame.K_LEFT:
                    cmd.angular.z = cmd.angular.z + w_max
                if event.key == pygame.K_RIGHT:
                    cmd.angular.z = cmd.angular.z - w_max

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_UP:
                    cmd.linear.x = cmd.linear.x - vel_max
                if event.key == pygame.K_DOWN:
                    cmd.linear.x = cmd.linear.x + vel_max
                if event.key == pygame.K_LEFT:
                    cmd.angular.z = cmd.angular.z - w_max
                if event.key == pygame.K_RIGHT:
                    cmd.angular.z = cmd.angular.z + w_max

        pub.publish(cmd)
        rate.sleep()
