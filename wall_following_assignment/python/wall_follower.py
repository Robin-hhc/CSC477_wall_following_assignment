#!/usr/bin/env python3
import rospy
import tf
from std_msgs.msg import String, Header, Float32
from geometry_msgs.msg import Twist
from math import sqrt, cos, sin, pi, atan2
import numpy
import sys

from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from wall_following_assignment.cfg import DynReconf1Config


class PID:
    def __init__(self, Kp, Td, Ti, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt

    def update_control(self, current_error, reset_prev=False):
        # todo: implement this
        self.prev_error = self.curr_error
        self.curr_error = current_error
        self.sum_error += current_error
        self.prev_error_deriv = self.curr_error_deriv
        self.curr_error_deriv = (current_error - self.prev_error) / self.dt
        if reset_prev:
            self.prev_error = 0
            self.sum_error = current_error
        self.control = self.Kp * (current_error + self.Td * self.curr_error_deriv + self.sum_error / self.Ti)

        return self.control

    def get_control(self):
        return self.control

    def srv_callback(self, config, level):
        #Call back funtion for server. Set four parameters to the attributes
        self.Kp = config.Kp
        self.Td = config.Td
        self.Ti = config.Ti
        self.dt = config.dt
        rospy.loginfo("server Kp: " + str(self.Kp) + ", Td: " + str(self.Td) + ", Ti: " + str(self.Ti))
        return config



class WallFollowerHusky:
    def __init__(self):
        rospy.init_node('wall_follower_husky', anonymous=True)

        self.forward_speed = rospy.get_param("~forward_speed")
        self.desired_distance_from_wall = rospy.get_param("~desired_distance_from_wall")
        self.hz = 50 

        # todo: set up the command publisher to publish at topic '/husky_1/cmd_vel'
        # using geometry_msgs.Twist messages
        self.cmd_pub = rospy.Publisher("/husky_1/cmd_vel", Twist, queue_size=100)

        # todo: set up the laser scan subscriber
        # this will set up a callback function that gets executed
        # upon each spinOnce() call, as long as a laser scan
        # message has been published in the meantime by another node
        self.laser_sub = rospy.Subscriber("/husky_1/scan", LaserScan, self.laser_scan_callback)
        rospy.loginfo("sub laser")

        #Set the PID for later calculation
        self.pid = PID(1, 13, 500, 0.1)

        
    def laser_scan_callback(self, msg):
        # todo: implement this
        # Populate this command based on the distance to the closest
        # object in laser scan. I.e. compute the cross-track error
        # as mentioned in the PID slides.

        # You can populate the command based on either of the following two methods:
        # (1) using only the distance to the closest wall
        # (2) using the distance to the closest wall and the orientation of the wall
        #
        # If you select option 2, you might want to use cascading PID control. 
  
        # cmd.angular.z = ???

        cte_pub = rospy.Publisher('/husky_1/cte', Float32, queue_size=100)
        e_t = min(msg.ranges) - self.desired_distance_from_wall
        cte_pub.publish(e_t)

        self.pid.update_control(e_t)
        v = self.pid.get_control()
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = v
        self.cmd_pub.publish(twist)
        return msg
            
    def run(self):
        rate = rospy.Rate(self.hz)
        #Set the server for Dynamic Reconfigure
        srv = Server(DynReconf1Config, self.pid.srv_callback)
        rospy.loginfo("final Kp: " + str(self.pid.Kp) + ", Td: " + str(self.pid.Td) + ", Ti: " + str(self.pid.Ti))
        while not rospy.is_shutdown():
            rate.sleep()

    
if __name__ == '__main__':
    wfh = WallFollowerHusky()
    wfh.run()


