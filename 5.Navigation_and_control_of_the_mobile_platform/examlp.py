#! /usr/bin/env python3
from os import lseek, name

from numpy.lib.function_base import select
import rospy, math, numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rospy import topics
from rospy.core import is_shutdown
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import atan2

class VelocityController():
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
        self.twist_msg = Twist()
        
        rospy.sleep(0.1)

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        self.twist_msg.linear.x = linear_velocity
        self.twist_msg.angular.z = angular_velocity
        self.cmd_vel.publish(self.twist_msg)

class OdometryReader():
    def __init__(self, topic):
        self.pose = {}
        self.topic = topic

        self.subscribe()
        rospy.sleep(0.1)

    def subscribe(self):
        self.subscriber = rospy.Subscriber(self.topic, Odometry, self.callback)

    def callback(self, msg):
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        (_, _, self.pose['theta']) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                            msg.pose.pose.orientation.y, 
                                                            msg.pose.pose.orientation.z, 
                                                            msg.pose.pose.orientation.w])

class LazerReader():
    def __init__(self, topic):
        self.laser_msg = LaserScan()
        self.topic = topic

        self.subscribe()
        rospy.sleep(0.1)

    def subscribe(self):
        self.subscriber = rospy.Subscriber(self.topic, LaserScan, self.callback)

    def callback(self, msg):
        self.laser_msg = msg

    def get_laser(self, pos):
        return self.laser_msg.ranges[pos]

    def print_laser(self):
        print("180: " + str(self.laser_msg.ranges[180]) +  "  ||  360: " + str(self.laser_msg.ranges[360]) + "  ||  540: " + str(self.laser_msg.ranges[540]))

    def print_laser(self, pos):
        print(str(pos) + ": " + str(self.laser_msg.ranges[pos]))

class RobotControl():
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        rospy.Rate(20)
        self.velocity = VelocityController('/cmd_vel')
        self.odometry = OdometryReader('/odom')
        self.laser = LazerReader('/laser/scan')

        self.current_coordinate = Point()
        self.current_theta = 0

        self.default_velocity = 0.6

    def update_current_pose(self):
        print(self.current_coordinate.x )
        self.current_coordinate.x = self.odometry.pose['x']
        self.current_coordinate.y = self.odometry.pose['y']
        self.current_theta = self.odometry.pose['theta']
    
    
    def move_to(self, goal):
        angle_to_goal = atan2(goal.y - self.current_coordinate.y, goal.x - self.current_coordinate.x)
        if abs(angle_to_goal - self.odometry.pose['theta']) > 0.05:
            self.velocity.move(0, 0.2)
        else:
            self.velocity.move(0.3, 0)
        

    def stop(self):
        self.velocity.move(0, 0)
        rospy.sleep(1)

if __name__ == '__main__':
    rc = RobotControl()
    rc.stop()
    rc.update_current_pose()
    goal = Point()
    goal.x = 1
    goal.y = 0

    while not rospy.is_shutdown():
        dx = goal.x - rc.odometry.pose['x']
        dy = goal.y - rc.odometry.pose['y']
        rho = np.sqrt(dx**2 + dy**2)

        if (rho < 0.1) :
            rc.stop()
            rc.update_current_pose()
            if goal.x < 2 :
                goal.x = goal.x + 1
                goal.y = 0
            else:
                goal.y = goal.y - 1
        else:
            rc.move_to(goal)

        rospy.sleep(0.1)
        