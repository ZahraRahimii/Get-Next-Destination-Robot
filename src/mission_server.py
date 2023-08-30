#!/usr/bin/python3

from __future__ import print_function

from next_dst.srv import GetNextDstSrv, GetNextDstSrvResponse
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import rospy
import random
import sys

class Mission:
    def __init__(self) -> None:
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, callback=self.odom_callback)
        self.path_publisher = rospy.Publisher('/path', Path, queue_size=10)
        self.path = Path()
        get_next_dst_server()
        
    def odom_callback(self, msg: Odometry):
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

def calculate_next(a):
    # next_a = a
    karan_bala = min(a+5, 10)
    karan_paeen = max(-10, a-5)
    random_num = random.uniform(karan_paeen, karan_bala)
    # max_x = random.uniform(10, a - 5)
    # print(min_x)
    # if random.uniform(0, 1) <= 0.5:
    #     next_a = min_x
    # else:
    #     next_a = max_x
    return random_num

def handle_get_nxt_dst(req):
    next_x = calculate_next(req.current_x)
    next_y = calculate_next(req.current_y)

    print("Returning x = [%s], y = [%s] so that next_x = [%s] and next_y = [%s]"%(req.current_x, req.current_y, next_x, next_y))
    resp = GetNextDstSrvResponse()
    resp.next_x = next_x
    resp.next_y = next_y
    return resp

def get_next_dst_server():
    rospy.init_node('mission_node')
    s = rospy.Service('get_next_dst', GetNextDstSrv, handle_get_nxt_dst)
    print("Ready to get next destination.")

if __name__ == "__main__":
    
    mission = Mission()

    rospy.spin()

