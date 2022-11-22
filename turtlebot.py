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


class AStar:
    def __init__(self, start, goal, obstacles):
        self.obstacles = obstacles
        self.start = start
        self.goal = goal
        pass

    def neighbors(self, current):
        # define the list of 4 neighbors
        neighbor_coords = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        return [(current[0] + nbr[0], current[1] + nbr[1]) for nbr in neighbor_coords]

    def heuristic_distance(self, candidate):
        dx = abs(candidate[0] - self.goal[0])
        dy = abs(candidate[1] - self.goal[1])
        return (dx + dy) + min(dx, dy) * (sqrt(2) - 2)

    def get_path_from_A_star(self):
        # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
        #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
        #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
        # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
        #   note that the path should contain the goal but not the start
        #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)]

        current = self.start  # Assign start to current (protects against empty set error)

        open_list = [(0, self.start)]
        closed_list = []
        path = []

        cost = {self.start: 0}  # Cost Dictionary
        parent = {self.start: "none"}  # Parent Dictionary

        # Check if goal is obstacle

        while open_list:
            open_list.sort()  # Sort list in ascending order
            current = open_list.pop(0)[1]
            closed_list.append(current)

            # Check if goal reached
            if current == self.goal:
                break

            for candidate in self.neighbors(current):
                if candidate in self.obstacles:  # Checked if candidate is not an obstacle
                    continue
                if candidate in closed_list:  # Checked if candidate has been visited
                    continue

                new_cost = cost[current] + 1
                if candidate not in cost or new_cost < cost[candidate]:
                    cost[candidate] = new_cost
                    parent[candidate] = current

                    final_cost = self.heuristic_distance(candidate) + cost[candidate]
                    open_list.append((final_cost, candidate))

        while current != self.start:
            path.append(current)
            current = parent[current]

        path.reverse()  # Generate waypoints by reversing path list

        return path


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
        error = checkBounds(self.set_point - current_value)
        P_term = self.Kp * error
        D_term = self.Kd * self.previous_error

        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point  # Set the setpoint
        self.check_Set_Point_Bounds()  # Ensure setpoint is within ROS angle domain
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
        self.velocity = 0.45

        # Controller variables
        self.controller = Controller(5.48)
        self.vel = Twist()  # Velocity X and Velocity Theta

        # AStar parameters
        self.start = (0, 0)
        self.goal = (9, 2)
        self.obstacles = [(1, 0), (1, 1), (4, 1), (4, 2), (4, 3), (5, 1), (5, 2), (5, 3), (8, 0), (8, -1)]
        self.a_star = AStar(self.start, self.goal, self.obstacles)

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

    def run(self):
        waypoints = self.a_star.get_path_from_A_star()
        waypoints.append(self.goal)

        for i in range(len(waypoints) - 1):
            self.move_to_point(waypoints[i], waypoints[i + 1])

    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint

        T = 2
        c = 10  # May need to be tweaked!!

        # Derive end velocities
        angle = atan2(next_waypoint[1] - current_waypoint[1],
                      next_waypoint[0] - current_waypoint[0])  # Calculate velocity angle

        # Tweaked for non-origin endpoints
        if current_waypoint[0] != next_waypoint[0] and current_waypoint[1] != next_waypoint[1]:
            v_end_x = self.velocity * cos(angle)
            v_end_y = self.velocity * sin(angle)
        else:
            v_end_x = 0
            v_end_y = 0

        aX = self.polynomial_time_scaling_3rd_order(self.previous_waypoint.item(0), self.previous_velocity.item(0),
                                                    current_waypoint[0], v_end_x,
                                                    T)  # Coefficients for 3rd order polynomial for X coordinates
        aY = self.polynomial_time_scaling_3rd_order(self.previous_waypoint.item(1), self.previous_velocity.item(1),
                                                    current_waypoint[1], v_end_y,
                                                    T)  # Coefficients for 3rd order polynomial for Y coordinates

        for i in range(c * T):
            t = i * 0.1

            # Why are these values unnecessary?
            posX = aX.item(3) + aX.item(2) * t + aX.item(1) * pow(t, 2) + aX.item(0) * pow(t, 3)
            posY = aY.item(3) + aY.item(2) * t + aY.item(1) * pow(t, 2) + aY.item(0) * pow(t, 3)

            velX = aX.item(2) + 2 * aX.item(1) * t + 3 * aX.item(0) * pow(t, 2)
            velY = aY.item(2) + 2 * aY.item(1) * t + 3 * aY.item(0) * pow(t, 2)

            # Update controller set point and velocity with velocity values retrieved from polynomial fcn
            self.controller.setPoint(atan2(velY, velX))

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
