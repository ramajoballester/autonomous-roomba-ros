#!/usr/bin/env python

import rospy
import sys
import time
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalStatusArray
from robotics.msg import Task, Bid, AuctionResult
from std_msgs.msg import Int16

robot_ns = 'roomba'
n_robots = 4
auction_time = 1.0              # Seconds
max_n_robots_idle = 0           # Max robots without task at a time

class Auction():
    def __init__(self, n_robots, robot_ns, auction_time):
        # Target positions
        self.x1 = [-4.0, 0.0, -4.0, 6.0, 4.0, -1.0, 3.0, 8.0]*10
        self.y1 = [-3.0, 3.0, 1.0, -3.0, 1.0, -4.0, -4.0, 2.0]*10
        self.x2 = [-1.0, 4.0, 8.0, 3.0, 0.0, -4.0, 6.0, -4.0]*10
        self.y2 = [-4.0, 1.0, 2.0, -4.0, 3.0, -3.0, -3.0, 1.0]*10

        self.n_tasks = len(self.x1)
        self.done_tasks = 0
        self.n_robots = n_robots
        self.bids = [-1]*n_robots
        self.robot_ns = robot_ns
        self.states = [-1]*n_robots
        self.auction_time = auction_time

        # Published topics
        self.auction_pub = rospy.Publisher('auction', Task, queue_size=1)
        self.result_pub = rospy.Publisher('auction_result', AuctionResult, queue_size=1)

        # Subscribed topics (for each robot)
        for i in range(1, self.n_robots + 1):
            rospy.Subscriber(str(robot_ns) + str(i) + '/bid', Bid, self.bid_callback, i-1)

        for i in range(1, self.n_robots + 1):
            rospy.Subscriber(str(robot_ns) + str(i) + '/bidder_ready',
                                Int16, self.state_callback, i-1)

    def bid_callback(self, msg, i):     # Get robots bids
        self.bids[i] = msg.bid

    def state_callback(self, msg, i):   # Get robots states
        self.states[i] = msg.data

    def start_auction(self):
        for i in range(1, self.n_robots + 1):
            if auction.done_tasks < auction.n_tasks:    # If there are tasks left
                self.bids = [-1]*n_robots
                task = Task()
                task.x1 = self.x1[self.done_tasks]
                task.y1 = self.y1[self.done_tasks]
                task.x2 = self.x2[self.done_tasks]
                task.y2 = self.y2[self.done_tasks]
                self.auction_pub.publish(task)
                time.sleep(self.auction_time)       # Auction time

                print(self.bids)

                min_bid = 1e6
                min_bid_id = -1
                # Select best bid (lowest)
                for j in range(1, self.n_robots + 1):
                    bid = self.bids[j-1]
                    if bid == -1:                           # If robot has not bid
                        print(str(self.robot_ns) + str(j) + ' has not bid')
                    else:
                        if bid < min_bid:
                            min_bid = bid
                            min_bid_id = j

                if min_bid_id != -1:        # If a robot can do task:
                    print(min_bid)
                    result = AuctionResult()
                    result.task = task
                    result.robot_id = str(self.robot_ns) + str(min_bid_id)
                    self.result_pub.publish(result) # Assign task to robot
                    self.done_tasks += 1

                time.sleep(0.2)

        print('Auction finished')
        print('')


if __name__ == '__main__':
    rospy.init_node('auctioneer')
    rate = rospy.Rate(5.0)

    print('Auctioneer ready')
    print('')

    # Configuration via terminal (optional)
    # if len(sys.argv) > 1:
    #     robot_ns = sys.argv[1]
    #
    # if len(sys.argv) > 2:
    #     n_robots = int(sys.argv[2])

    auction = Auction(n_robots, robot_ns, auction_time)
    finished = False

    # While not finished, do auction; else print time elapsed
    while (not rospy.is_shutdown()) and (not finished):
        if auction.done_tasks < auction.n_tasks:
            if auction.states.count(1) > max_n_robots_idle:
                if auction.done_tasks == 0:
                    t_init = rospy.get_time()
                auction.start_auction()

        else:
            if auction.states == [1]*n_robots:
                finished = True
                t_end = rospy.get_time()
                print('')
                print('Total time: ' + str(t_end-t_init) + ' seconds')

        rate.sleep()
