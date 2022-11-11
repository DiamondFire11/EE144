#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.run()


    def run(self):
        vel = Twist()
        
        vel.linear.x = 0
        vel.angular.z = 0.15
        for correction in range(7):
            self.vel_pub.publish(vel)
            self.rate.sleep()           

        for edge in range(4):
            vel.linear.x = 0.25
            vel.angular.z = 0
            #while not rospy.is_shutdown():  # uncomment to use while loop
            for i in range(159):
                self.vel_pub.publish(vel)
                self.rate.sleep()
            
            vel.linear.x = 0
            vel.angular.z = 0.15
            #while not rospy.is_shutdown():  # uncomment to use while loop
            for i in range(110):
                self.vel_pub.publish(vel)
                self.rate.sleep()
        

if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
