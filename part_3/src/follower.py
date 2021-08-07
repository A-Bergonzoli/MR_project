#!/usr/bin/env python3

'''
This node implements the leader-follower dynamic, in which the leader (a turtlebot3_waffle) is
responsible for the path planning (achieved thanks to the Nav Stack) and the follower tracks the said trajectory
'''


from numpy.lib.function_base import select
import rospy
import actionlib

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math

from queue_implementation import *
from control import *


class Follower(object):

    '''attributes'''
    twist_msg = Twist()
    # pose of the follower
    q = []
    # pose of the leader (TB3_waffle)
    q0 = []
    # velocities of the leader (TB3_waffle)
    v0 = 0
    w0 = []
    # radius of the sensing/communication disk
    R = 2
    # start position of the leader
    q0_i = [-2, 1]
    # goal position of the leader
    q0_f = [1.9, -1.8, 0.0]

    '''methods'''
    # Constructor
    def __init__(self):
        # Initialize node
        rospy.init_node('Follower_node')
        # This node is a publisher on the following topic:
        self.vel_pub = rospy.Publisher('/Follower/DD_controller/cmd_vel', Twist, queue_size=1)
        # This node is a subsciber to the following topics:
        self.turtle_sub = rospy.Subscriber('/Follower/ground_truth/state', Odometry, self.cb_follower_pose)
        self.tb3_waffle_sub = rospy.Subscriber('/leader/odom', Odometry, self.cb_leader_pose)
        self.vel_sub = rospy.Subscriber('/leader/cmd_vel', Twist, self.cb_leader_vel)


    # Callbacks:
    # Poses
    def cb_follower_pose(self, msg):
        x = round(msg.pose.pose.position.x, 4)
        y = round(msg.pose.pose.position.y, 4)
        theta = round(self.get_heading(msg.pose.pose.orientation), 4)
        self.q = np.array([x, y, theta])
    def cb_leader_pose(self, msg):
        x = round(msg.pose.pose.position.x, 4)
        y = round(msg.pose.pose.position.y, 4)
        theta = round(self.get_heading(msg.pose.pose.orientation), 4)
        self.q0 = np.array([x, y, theta])
    # Velocities
    def cb_leader_vel(self, msg):
        self.v0 = msg.linear.x
        self.w0 = msg.angular.z

    # Utility function to compute hading from quaternion
    def get_heading(self, quaternion):
        roll, pitch, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw

    # Get info from callbacks:
    # Poses:
    def get_follower_pose(self):
        return np.array([self.q[0], self.q[1], self.q[2]])
    def get_leader_pose(self):
        return [self.q0[0], self.q0[1], self.q0[2]]
    # Velocoties
    def get_leader_vel(self):
        return [self.v0, self.w0]

    # Compute position of the output point B to implement io-linearization
    def get_point_coordinate(self, b):
        pose = self.get_follower_pose()
        x = pose[0]
        y = pose[1]
        theta = pose[2]
        # point B coordinates
        y1 = x + b * np.cos(theta)
        y2 = y + b * np.sin(theta)
        return [y1, y2, theta]

    # Utility funciton for the follower
    def has_reached_start(self, follower):
        dist = np.linalg.norm( np.array([follower[0], follower[1]]) - np.array([self.q0_i[0], self.q0_i[1]]) )
        if (dist <= 0.2):
            return True
        else:
            return False
    # Utility function for the follower
    def has_reached_goal(self, follower):
        dist = np.linalg.norm( np.array([follower[0], follower[1]]) - np.array([self.q0_f[0], self.q0_f[1]]) )
        if (dist <= 0.4):
            return True
        else:
            return False

    # Compute the tracking error to implement non-linear control
    def get_error(self, follower, leader):
        # slide 80 LDC
        (xf, yf, thf) = follower
        (xl, yl, thl) = leader
        # compute error
        e1 = (xl - xf) * np.cos(thf) + (yl - yf) * np.sin(thf)
        e2 = -(xl - xf) * np.sin(thf) + (yl - yf) * np.cos(thf)
        e3 = thl - thf

        return np.array([e1, e2, e3])


    def follow(self):
        
        # flag that indicates if the follower can start tracking the leader's trajectory
        start_reached = False 
        # flag that indicates if the follower has reached the goal position
        goal_reached = False

        b = 0.1 # distance of point B from the point P (point of contact of the robot)

        rospy.sleep(0.1) # sleep to setup q and q0
        rate = rospy.Rate(10)

        # initialize buffer for leader pose and velocities
        buffer = Queue()

        while (True):

            # get follower pose
            f_pose = self.get_follower_pose()
            # get leader info
            l_pose = self.get_leader_pose()
            l_velocity = self.get_leader_vel()
            # get point B coordinates (+ theta)
            (y1, y2, theta) = self.get_point_coordinate(b)

            # set flags
            if self.has_reached_start(f_pose): start_reached = True
            if self.has_reached_goal(f_pose): goal_reached = True

            if goal_reached:
                break
            else:
                if not start_reached:                                     # the follower IS NOT tracking the leader's path (has not yet reached the leader's start position)
                    # insert leader's info in the buffer
                    buffer.enQ([l_pose, l_velocity])
                    # get follower velocities
                    (v, w) = io_linearization_control_law(y1, y2, theta, self.q0_i[0], self.q0_i[1], 0.1, 0.0, b, False)

                else:                                                    # the follower IS tracking the leader's path
                    if buffer.is_empty():
                        # get follower velocities
                        (v, w) = nonLinear_control_law(err, l_velocity[0], l_velocity[1])
                    else:
                        # get leader's info from buffer
                        desired_pose = buffer.peek_first()[0]
                        desired_vel = buffer.peek_first()[1]
                        # get follower velocities
                        err = self.get_error(f_pose, desired_pose)
                        (v, w) = nonLinear_control_law(err, desired_vel[0], desired_vel[1])
                        # remove leader's info from the buffer
                        buffer.deQ()
                        # insert leader's info in the buffer
                        buffer.enQ([l_pose, l_velocity])
                
                # publish velocities and move robot
                self.twist_msg.linear.x = float(0.5*v)
                self.twist_msg.angular.z = -w              
                self.vel_pub.publish(self.twist_msg)

                print("linear:{} and angular:{}".format(self.twist_msg.linear.x , self.twist_msg.angular.z)) #debug 
            
                # sleep to implement a 10Hz loop
                rate.sleep() 

        # stop robot
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0              
        self.vel_pub.publish(self.twist_msg)



if __name__ == "__main__":
    follow_obj = Follower()

    client = actionlib.SimpleActionClient('/leader/move_base', MoveBaseAction)
    client.wait_for_server()

    # send goal to leader
    quaternion = quaternion_from_euler(0.0, 0.0, math.pi/2)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = follow_obj.q0_f[0]
    goal.target_pose.pose.position.y = follow_obj.q0_f[1]
    goal.target_pose.pose.position.z = follow_obj.q0_f[2]
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    client.send_goal(goal)

    follow_obj.follow()
