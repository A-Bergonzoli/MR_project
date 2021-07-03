#!/usr/bin/env python3

'''
This node will operate a rendez vous of the spawned robots.
'''

import numpy as np
from ctrl_law import *
from tf.transformations import euler_from_quaternion
from scipy.spatial import distance

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Agent_2():

    ### ATTRIBUTES ####

    q2 = [-5, 5, 0]
    q1 = [5, 5, 0]
    q3 = [5, -5, 0]
    q4 = [-5, -5, 0]
    q5 = [0, 1, 0]
    twist_msg = Twist()
    R = 3    #[m] radius of the sensing/communication disk


    ### METHODS ###

    # Construsctor
    def __init__(self):
        # Initialize node
        rospy.init_node('agent_2')
        # Printo sulla shell
        rospy.loginfo("Starting agent_2!")
        # This node is a publisher on his /cmd_vel topic
        self.cmd_vel2_pub = rospy.Publisher('/Turtle2/DD_controller/cmd_vel', Twist, queue_size=1)
        # This node is a subscriber to all the /turtle/odom (ground_truth/state) topics
        self.pose2_sub = rospy.Subscriber('/Turtle2/ground_truth/state', Odometry, self.cb1)
        self.pose1_sub = rospy.Subscriber('/Turtle1/ground_truth/state', Odometry, self.cb2)
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

 
    # Tells us whether or not a given agent is neighbour of agent_2
    def isNeighbour(self, agent):
        rospy.sleep(0.1)
        pose = self.get_pose1()
        dist = distance.euclidean( [pose[0], pose[1]], [agent[0], agent[1]] )
        if (dist <= self.R):
            return True
        else:
            return False


    # Defines the neighbour set for agent_2
    def define_neighbours(self, agent2, agent3, agent4):
        neighbour_set = []
        if (self.isNeighbour(agent2)):
            neighbour_set.append(agent2)
        if (self.isNeighbour(agent3)):
            neighbour_set.append(agent3)
        if (self.isNeighbour(agent4)):
            neighbour_set.append(agent4)
        return neighbour_set
        

    # Send velocities
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
            N_2 = self.define_neighbours(pose2, pose3, pose4)
            # Compute inputs:
            if (len(N_2) != 0):
                (v, w) = rv_control_law_ar2(pose1, N_2)
            else:
                v = 0.0
                w = 0.0
            # Publish velocities on cmd_vel_i topics
            self.twist_msg.linear.x = -v
            self.twist_msg.angular.z = -w
            self.cmd_vel2_pub.publish(self.twist_msg)

            # sleep for 0.1s
            rospy.sleep(0.1)


if __name__ == "__main__":
    # Instance an object of the 'rendezvous' class
    ob = Agent_2()

    # rate for the loop cycle (Hz)
    rate = rospy.Rate(10)

    rospy.sleep(3)

    # loop cycle
    while not rospy.is_shutdown():
        ob.rendezvous_control()
 
        rate.sleep()  # to implement a 10Hz loop
