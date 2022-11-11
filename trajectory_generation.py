#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

def checkBounds(error):
    if error < -pi:
        return error + 2 * pi

    if error > pi:
        return error - 2 * pi

    return error


class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point  # reference (desired value)
        self.previous_error = 0

    def check_Set_Point_Bounds(self):
        if self.set_point > pi:
            self.set_point = self.set_point - 2 * pi

        if self.set_point < -pi:  # This case should be unreachable, here for safety
            self.set_point = self.set_point + 2 * pi

    def update(self, current_value):
        # calculate error, P_term, and D_term
        error = checkBounds(current_value - self.set_point)
        P_term = self.Kp * error
        D_term = self.Kd * self.previous_error

        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.check_Set_Point_Bounds()
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

        # Waypoint variables
        self.previous_waypoint = np.matrix([0, 0])
        self.previous_velocity = np.matrix([0, 0])
        self.velocity = 0.5

        # Controller variables
        self.controller = Controller(1.0)
        self.vel = Twist()  # Velocity X and Velocity Theta

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

    def run(self):
        waypoints = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5], \
                     [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0], \
                     [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]
        for i in range(len(waypoints) - 1):
            self.move_to_point(waypoints[i], waypoints[i + 1])

    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint

        T = 2
        c = 10  # May need to be tweaked!!

        # X and Y end velocity (dx/dt | dy/dt)
        v_end_x = (current_waypoint[0] - next_waypoint[0]) / T
        v_end_y = (current_waypoint[1] - next_waypoint[1]) / T

        aX = self.polynomial_time_scaling_3rd_order(self.previous_waypoint.item(0), self.previous_velocity.item(0), current_waypoint[0], v_end_x, T)  # Coefficients for 3rd order polynomial for X coordinates
        aY = self.polynomial_time_scaling_3rd_order(self.previous_waypoint.item(1), self.previous_velocity.item(1), current_waypoint[1], v_end_y, T)  # Coefficients for 3rd order polynomial for Y coordinates

        for i in range(c * T):
            t = i * 0.1
            posX = aX.item(3) + aX.item(2) * t + aX.item(1) * pow(t, 2) + aX.item(0) * pow(t, 3)
            posY = aY.item(3) + aY.item(2) * t + aY.item(1) * pow(t, 2) + aY.item(0) * pow(t, 3)

            velX = aX.item(2) + 2 * aX.item(1) * t + 3 * aX.item(0) * pow(t, 2)
            velY = aY.item(2) + 2 * aY.item(1) * t + 3 * aY.item(0) * pow(t, 2)

            # Update controller set point and velocity with values retrieved from polynomial fcn
            self.controller.setPoint(atan2(posY, posX))

            self.vel.linear.x = sqrt(pow(velX, 2) + pow(velY, 2))
            self.vel.angular.z = self.controller.update(self.pose.theta)

            # Tell robot to execute velocity
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        self.previous_waypoint = np.matrix([current_waypoint[0], current_waypoint[1]])
        self.previous_velocity = np.matrix([v_end_x, v_end_y])
        pass

    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        polynomialMtx = np.matrix([[0, 0, 0, 1], [pow(T, 3), pow(T, 2), T, 1], [0, 0, 1, 0], [3 * pow(T, 2), 2 * T, 1, 0]])
        constraintMtx = np.matrix([p_start, p_end, v_start, v_end])
        return np.linalg.inv(polynomialMtx) * np.transpose(constraintMtx)

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
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
            rospy.loginfo("odom: x=" + str(self.pose.x) + \
                          ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    whatever = Turtlebot()
