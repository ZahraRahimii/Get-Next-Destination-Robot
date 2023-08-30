#!/usr/bin/python3

import sys
import rospy
from next_dst.srv import GetNextDstSrv
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from math import radians, pi, atan2, sqrt, pow
import numpy as np


class Control:

    def __init__(self) -> None:
        
        rospy.init_node("controller_node" , anonymous=True)
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist , queue_size=10)
        # self.linear_speed = rospy.get_param(rospy.get_namespace() + "linear_speed") # m/s
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        # self.linear_speed = 0.2 # m/s
        self.angular_speed = 0.3 # r/s
        # self.epsilon = 0.05
        
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        self.curr_x = 0.0; self.next_x = self.curr_x
        self.curr_y = 0.0; self.next_y = self.curr_y

    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        self.x_pose = msg.pose.pose.position.x
        self.y_pose = msg.pose.pose.position.y

        self.curr_x = self.x_pose
        self.curr_y = self.y_pose
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        self.yaw = yaw
        return yaw
    
    def rotation(self, goal_angle, epsilon):
        r = rospy.Rate(self.angular_speed * 30)
        self.make_twist(0, 0)
        r.sleep()

        twist = Twist()
        
        direction = True
        if goal_angle > 0.1:
            direction = False
        # self.state = self.ROTATE
        if direction:
            twist.angular.z = self.angular_speed 
        else: 
            twist.angular.z = -self.angular_speed 
        remaining = abs(goal_angle)
        prev_angle = self.get_heading()
        
        self.cmd_publisher.publish(twist)

        while (remaining) >= epsilon:
            current_angle = self.get_heading()
            delta = abs(prev_angle - current_angle)
            remaining -= (delta)
            prev_angle = current_angle
            r.sleep()
        self.cmd_publisher.publish(twist)

    def move_forward(self, distance_to_dst):
        r = rospy.Rate(self.linear_speed * 10)
        speed_step = 0.01
        speed = 0.1
        self.make_twist(speed, 0)
        
        self.state = self.GO
        while distance_to_dst >= 0.7:
            msg = rospy.wait_for_message("/odom" , Odometry)

            x_pose = msg.pose.pose.position.x
            y_pose = msg.pose.pose.position.y

            distance_to_dst = two_points_distance(x_pose, y_pose, self.next_x, self.next_y)
            print(f'distance left: {distance_to_dst}')
            self.curr_x = x_pose
            self.curr_y = y_pose

            if speed + speed_step < self.linear_speed:
                speed += speed_step
                self.make_twist(speed, 0)
            
            r.sleep()

    def make_twist(self, linear_speed, angular_speed):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmd_publisher.publish(twist)
    
    def updata_pos(self):
        msg = rospy.wait_for_message("/odom" , Odometry)

        x_pose = msg.pose.pose.position.x
        y_pose = msg.pose.pose.position.y

        self.curr_x = x_pose
        self.curr_y = y_pose


    def run(self):
        # print('hello there', self.get_heading())
        print('\n\n\n')

        # while not rospy.is_shutdown():
        for i in range(5):
            
            r = rospy.Rate(self.linear_speed * 10)

            # yaw = 1.7792088820999849e-07
            print(f'\nIteration {i + 1}:\n')
            # if self.get_heading()
            # if i != 0:
            #     self.make_twist(0, 0)
            #     r.sleep()
            #     r.sleep()
            #     r.sleep()
            #     epsilon = 0.01  
            #     print('Position Reset')
            #     # self.rotation(yaw, self.get_heading(), epsilon)
            
            self.next_x, self.next_y = get_nxt_dst(self.curr_x, self.curr_y)
            # self.next_x = -2
            # self.next_y = 12
            print(f"\n\nnext x \t{self.next_x}")
            print(f"\n\nnext y \t{self.next_y}\n\n")
            
            inc_x = self.next_x - self.curr_x
            inc_y = self.next_y - self.curr_y
            
            angle_to_dst = atan2(inc_y, inc_x)
            theta = 0

            if angle_to_dst > pi:
                angle_to_dst = (angle_to_dst + 2 * pi)
            elif angle_to_dst < -pi:
                angle_to_dst = (angle_to_dst - 2 * pi)

            yaw = self.get_heading()
            remaining = yaw - angle_to_dst
            # remaining = self.yaw - (angle_to_dst - theta)

            # if remaining < -pi:
            #     remaining = (remaining + 2 * pi)
            # elif remaining > pi:
            #     remaining = (remaining - 2 * pi)

            # theta = 0

            'Roatate to the target angle'
            if abs(remaining) > 0.01:  
                epsilon = 0.01
                self.rotation(remaining, epsilon)

            self.make_twist(0, 0)
            r.sleep()
            r.sleep()

            'Move forward to the targer point with telorance of 2'
            distance_to_dst = two_points_distance(self.curr_x, self.curr_y, self.next_x, self.next_y)

            if distance_to_dst > 0.1:
                self.move_forward(distance_to_dst)
            
            self.updata_pos()
            self.make_twist(0, 0)
            r.sleep()
            r.sleep()
            r.sleep()
    
def two_points_distance(x1, y1, x2, y2):
        return sqrt(pow((x2-x1), 2) + pow((y2-y1), 2))

def get_nxt_dst(x, y):
    rospy.wait_for_service('get_next_dst')
    try:
        add_two_ints = rospy.ServiceProxy('get_next_dst', GetNextDstSrv)
        resp1 = add_two_ints(x, y)
        return (resp1.next_x, resp1.next_y)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
 
    controller = Control()
    controller.run()
 