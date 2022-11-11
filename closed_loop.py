#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

def checkBounds(set_point, curr_val):
    if(curr_val > pi):
        corrected_error = set_point - (curr_val + pi)
        return corrected_error - 2*pi
        
    if(curr_val < -pi):
        corrected_error = set_point - (curr_val - pi)
        return corrected_error + 2*pi
    
    return set_point - curr_val


def check_Set_Point_Bounds(set_point):
    if(set_point > pi):
        return set_point - 2*pi
        
    if(set_point < -pi): #This case should be unreachable, here for safety
        return set_point + 2*pi
    
    return set_point


class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # calculate error, P_term, and D_term
        error = current_value - self.set_point
        P_term = self.Kp * error
        D_term = self.Kd * (self.previous_error)

        self.previous_error = error
        return P_term + D_term
        
    def setPoint(self, set_point):
        self.set_point = check_Set_Point_Bounds(set_point)
        self.previous_error = 0
    
    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D


class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()
        
        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


    def run(self):
        vel = Twist()
        
        track_theta = Controller(1, 0, 0)
        waypoints = [[4,0], [4,4], [0,4], [0,0]]
        setTheta = 0
        
        threshold = 0.06

        for edge in waypoints:
            while(sqrt((edge[0] - self.pose.x)**2 + (edge[1] - self.pose.y)**2) > threshold):
                vel.linear.x = 0.55

                error = track_theta.update(self.pose.theta)
                if(error < 0):
                    vel.angular.z = abs(error)*2.5
                else:
                    vel.angular.z = 0

                #Tell robot to execute velocity
                self.vel_pub.publish(vel)
                self.rate.sleep()

            #Align robot along waypoint axis
            setTheta = check_Set_Point_Bounds((setTheta + pi/2))
            track_theta.setPoint(setTheta)
            while(abs(self.pose.theta - setTheta) > threshold):
                vel.linear.x = 0
                vel.angular.z = abs(track_theta.update(self.pose.theta))

                #Tell robot to execute velocity
                self.vel_pub.publish(vel)
                self.rate.sleep()
        pass


    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            # display (x, y, theta) on the terminal
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    whatever = Turtlebot()
