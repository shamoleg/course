#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from math import radians, copysign, sqrt, pow, pi, atan2
import PyKDL

class RobotControl():

    def __init__(self, robot_name="turtlebot"):
        rospy.init_node('robot_control_node')

        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()        

        self.laser_subscriber = rospy.Subscriber('/laser/scan', LaserScan, self.laser_callback)
        self.laser_msg = LaserScan() 
        
        self.rate = rospy.Rate(30)
        self.cmd = Twist()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.tf_listener = tf.TransformListener()
        self.odom_frame = '/odom'
        self.base_frame = '/base_link'


    def publish_once_in_cmd_vel(self):
        self.vel_publisher.publish(self.cmd)

    def laser_callback(self, msg):
        self.laser_msg = msg

    def get_laser(self, pos):
        return self.laser_msg.ranges[pos]

    def get_laser_full(self):
        return self.laser_msg.ranges

    def stop_robot(self):
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.publish_once_in_cmd_vel()

    def move_forward(self):

        self.cmd.linear.x = 0.5
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        self.publish_once_in_cmd_vel()


    def turn(self, clockwise, speed, time):
        # добавить реализацию аналогично предидущем методам
        pass

    def get_odom(self):

        # Get the current transform between the odom and base frames
        tf_ok = 0
        while tf_ok == 0 and not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform('/base_link', '/odom', rospy.Time(), rospy.Duration(1.0))
                tf_ok = 1
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                pass

        try:
            (trans, rot)  = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

    def rotate(self, degrees):

        position = Point()

        # Get the current position
        (position, rotation) = self.get_odom()

        # Set the movement command to a rotation
        if degrees > 0:
            self.cmd.angular.z = 0.3
        else:
            self.cmd.angular.z = -0.3

        # Track the last angle measured
        last_angle = rotation
        
        # Track how far we have turned
        turn_angle = 0

        goal_angle = radians(degrees)

        # Begin the rotation
        while abs(turn_angle + self.angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.vel_publisher.publish(self.cmd) 
            self.rate.sleep()
            
            # Get the current rotation
            (position, rotation) = self.get_odom()
            
            # Compute the amount of rotation since the last lopp
            delta_angle = self.normalize_angle(rotation - last_angle)
            
            turn_angle += delta_angle
            last_angle = rotation
        
        self.stop_robot()

    def quat_to_angle(self, q):
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        rot = atan2(sinr_cosp, cosr_cosp)
        return rot
        
    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res    


if __name__ == '__main__':
    
    robotcontrol_object = RobotControl()
    try:
        robotcontrol_object.move_straight()

    except rospy.ROSInterruptException:
        pass