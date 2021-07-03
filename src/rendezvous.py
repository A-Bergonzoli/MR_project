#!/usr/bin/env python3

'''
This node will operate a rendez-vous of the spawned robots.
The particular point depends only on the initial pose of the robots (it's the average).
'''

import numpy as np
from ctrl_law import *
from tf.transformations import euler_from_quaternion

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class rendezvous():

    ### ATTRIBUTES ####

    q1 = [-5, 5, 0]
    q2 = [5, 5, 0]
    q3 = [5, -5, 0]
    q4 = [-5, -5, 0]
    twist_msg1 = Twist()
    twist_msg2 = Twist()
    twist_msg3 = Twist()
    twist_msg4 = Twist()


    ### METHODS ###

    # Construsctor
    def __init__(self):
        # Initialize node
        rospy.init_node('rendez_vous')
        # Printo sulla shell
        rospy.loginfo("Starting rendez-vous operation!")
        # This node is a publisher on all the /turtle/cmd_vel topics
        self.cmd_vel1_pub = rospy.Publisher('/Turtle1/DD_controller/cmd_vel', Twist, queue_size=1)
        self.cmd_vel2_pub = rospy.Publisher('/Turtle2/DD_controller/cmd_vel', Twist, queue_size=1)
        self.cmd_vel3_pub = rospy.Publisher('/Turtle3/DD_controller/cmd_vel', Twist, queue_size=1)
        self.cmd_vel4_pub = rospy.Publisher('/Turtle4/DD_controller/cmd_vel', Twist, queue_size=1)
        # This node is a subscriber to all the /turtle/odom (ground_truth/state) topics
        self.pose1_sub = rospy.Subscriber('/Turtle1/ground_truth/state', Odometry, self.cb1)
        self.pose2_sub = rospy.Subscriber('/Turtle2/ground_truth/state', Odometry, self.cb2)
        self.pose3_sub = rospy.Subscriber('/Turtle3/ground_truth/state', Odometry, self.cb3)
        self.pose4_sub = rospy.Subscriber('/Turtle4/ground_truth/state', Odometry, self.cb4)

    # Callbacks
    def cb1(self, msg):
        x = round(msg.pose.pose.position.x,4)
        y = round(msg.pose.pose.position.y,4)
        th = round(self.getHeading(msg.pose.pose.orientation),4)
        self.q1 = np.array([x, y, th])
        return self.q1
    def cb2(self, msg):
        x = round(msg.pose.pose.position.x,4)
        y = round(msg.pose.pose.position.y,4)
        th = round(self.getHeading(msg.pose.pose.orientation),4)
        self.q2 = np.array([x, y, th])
        return self.q2    
    def cb3(self, msg):
        x = round(msg.pose.pose.position.x,4)
        y = round(msg.pose.pose.position.y,4)
        th = round(self.getHeading(msg.pose.pose.orientation),4)
        self.q3 = np.array([x, y, th])
        return self.q3
    def cb4(self, msg):
        x = round(msg.pose.pose.position.x,4)
        y = round(msg.pose.pose.position.y,4)
        th = round(self.getHeading(msg.pose.pose.orientation),4)
        self.q4 = np.array([x, y, th])
        return self.q4

    # Compute angle from quaternion
    def getHeading(self, quaternion_pose):
        tmp = [quaternion_pose.x,
                quaternion_pose.y,
                quaternion_pose.z,
                quaternion_pose.w]
        roll, pitch, yaw = euler_from_quaternion(tmp)
        theta = yaw
        return theta

    # Get poses from callbacks
    def get_pose1(self):
        x1 = self.q1[0]
        y1 = self.q1[1]
        th1 = self.q1[2]
        return np.array([x1, y1, th1])
    def get_pose2(self):
        x2 = self.q2[0]
        y2 = self.q2[1]
        th2 = self.q2[2]
        return np.array([x2, y2, th2])
    def get_pose3(self):
        x3 = self.q3[0]
        y3 = self.q3[1]
        th3 = self.q3[2]
        return np.array([x3, y3, th3])
    def get_pose4(self):
        x4 = self.q4[0]
        y4 = self.q4[1]
        th4 = self.q4[2]
        return np.array([x4, y4, th4])

    # Send velocities on the respective topics
    def rendezvous_control(self):

        while (1==1):

            # sleep for 0.1s
            rospy.sleep(0.1)
            # Get poses
            pose1 = self.get_pose1()
            pose2 = self.get_pose2()
            pose3 = self.get_pose3()
            pose4 = self.get_pose4()          
            # Neighbours set
            N_1 = [pose2, pose4]  # list of neighbours of turtle 1
            N_2 = [pose1, pose3]  # list of neighbours of turtle 2
            N_3 = [pose2, pose4]  # list of neighbours of turtle 3
            N_4 = [pose1, pose3]  # list of neighbours of turtle 4
            # Compute inputs:
            # for turtle1           
            (v1, w1) = rv_control_law_ar1(pose1, N_1)
            # for turtle2
            (v2, w2) = rv_control_law_ar1(pose2, N_2)
            # for turtle3
            (v3, w3) = rv_control_law_ar1(pose3, N_3)
            # for turtle4
            (v4, w4) = rv_control_law_ar1(pose4, N_4)

            # Publish velocities on cmd_vel_i topics
            self.twist_msg1.linear.x = -v1
            self.twist_msg1.angular.z = -w1
            self.cmd_vel1_pub.publish(self.twist_msg1)
            self.twist_msg2.linear.x = -v2
            self.twist_msg2.angular.z = -w2
            self.cmd_vel2_pub.publish(self.twist_msg2)
            self.twist_msg3.linear.x = -v3
            self.twist_msg3.angular.z = -w3
            self.cmd_vel3_pub.publish(self.twist_msg3)
            self.twist_msg4.linear.x = -v4
            self.twist_msg4.angular.z = -w4
            self.cmd_vel4_pub.publish(self.twist_msg4)

            # sleep for 0.1s
            rospy.sleep(0.1)


if __name__ == "__main__":
    # Instance an object of the 'rendezvous' class
    ob = rendezvous()

    # rate for the loop cycle (Hz)
    rate = rospy.Rate(10)

    # loop cycle
    while not rospy.is_shutdown():
        ob.rendezvous_control()
 
        rate.sleep()  # to implement a 10Hz loop
