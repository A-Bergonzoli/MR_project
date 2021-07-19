#!/usr/bin/env python3

import rospy
import actionlib
from mr_project.msg import MoveGroupAction, MoveGroupGoal, MoveGroupResult, MoveGroupFeedback

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import numpy as np
import sympy as sp
import time
import math

from trajectory_generation import *
from ctrl_law import *
from formation_control import *


class GroupAction(object):

    '''attributes'''

    _feedback = MoveGroupFeedback()
    _result = MoveGroupResult()
    follower1 = Twist()
    follower2 = Twist()
    follower3 = Twist()
    follower4 = Twist()
    t = [] # time vector
    R = 3        # [m] radius of the communication/sensing disk
    # desired position fro the virtual leader
    x_d = []
    y_d = []
    # desired velcoities for the virtual leader
    v_d = []
    w_d = []
    # desired geometric pattern for the followers
    p1x = 0.5
    p1y = 0.5
    p2x = 0.5
    p2y = -0.5
    p3x = -0.5
    p3y = -0.5
    p4x = -0.5
    p4y = 0.5
    # obstacles parameters
    x_obs = 50
    y_obs = 50
    r_obs = 0.5  # [m] radius of the cylinder obstacle
    R_obs = 2.5  # [m] radius of the area where the repel force caused by the APF acts


    '''methods'''

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, MoveGroupAction, execute_cb=self.execute_cb, auto_start=False)
        self.turtle1_sub = rospy.Subscriber('/Turtle1/ground_truth/state', Odometry, self.cb1)
        self.turtle2_sub = rospy.Subscriber('/Turtle2/ground_truth/state', Odometry, self.cb2)
        self.turtle3_sub = rospy.Subscriber('/Turtle3/ground_truth/state', Odometry, self.cb3)
        self.turtle4_sub = rospy.Subscriber('/Turtle4/ground_truth/state', Odometry, self.cb4)
        self._as.start()
        rospy.loginfo('Server is ON')

    # Callbacks
    def cb1(self, msg):
        x = round(msg.pose.pose.position.x, 4)
        y = round(msg.pose.pose.position.y, 4)
        theta = round(self.get_heading(msg.pose.pose.orientation), 4)
        self.q1 = np.array([x, y, theta])
    def cb2(self, msg):
        x = round(msg.pose.pose.position.x, 4)
        y = round(msg.pose.pose.position.y, 4)
        theta = round(self.get_heading(msg.pose.pose.orientation), 4)
        self.q2 = np.array([x, y, theta])
    def cb3(self, msg):
        x = round(msg.pose.pose.position.x, 4)
        y = round(msg.pose.pose.position.y, 4)
        theta = round(self.get_heading(msg.pose.pose.orientation), 4)
        self.q3 = np.array([x, y, theta])
    def cb4(self, msg):
        x = round(msg.pose.pose.position.x, 4)
        y = round(msg.pose.pose.position.y, 4)
        theta = round(self.get_heading(msg.pose.pose.orientation), 4)
        self.q4 = np.array([x, y, theta])
    
    # Compute hading from quaternion
    def get_heading(self, quaternion):
        roll, pitch, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw%(2*math.pi)

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

    # Get a list of all poses
    def get_poses(self):
        pose1 = self.get_pose1()
        pose2 = self.get_pose2()
        pose3 = self.get_pose3()
        pose4 = self.get_pose4()
        return [pose1, pose2, pose3, pose4]


    # Utility function that tells us if a given agent_j (of pose_j) is a neighbour for agent_i (of pose_i)
    def isNeighbour(self, pose_i, pose_j):        
        dist = np.linalg.norm( np.array([pose_i[0], pose_i[1]]) - np.array([pose_j[0], pose_j[1]]) )
        if (dist <= self.R):
            return True
        else:
            return False

    # Defines the neighbour set for agent_i (of pose_i)
    def define_neighbours(self, pose_i, pose_j, pose_k, pose_h):
        neighbour_set = []
        if (self.isNeighbour(pose_i, pose_j)):
            neighbour_set.append(pose_j)
        if (self.isNeighbour(pose_i, pose_k)):
            neighbour_set.append(pose_k)
        if (self.isNeighbour(pose_i, pose_h)):
            neighbour_set.append(pose_h)
        return neighbour_set

    # Compute the average distance of the agents from the origin
    def compute_average_norm(self, pose_list):       
        norm1 = np.linalg.norm( [pose_list[0][0], pose_list[0][1]] )
        norm2 = np.linalg.norm( [pose_list[1][0], pose_list[1][1]] )
        norm3 = np.linalg.norm( [pose_list[2][0], pose_list[2][1]] )
        avg_norm = ( norm1 + norm2 + norm3 ) / 4
        return avg_norm

    # Defines the connection graph programmatically choosing the agents by assigning the follower(1-4) roles
    def define_graph(self, pose1, pose2, pose3, pose4):
        pose_list = [
            [pose1, 1, ''],
            [pose2, 2, ''],
            [pose3, 3, ''],
            [pose4, 4, '']
            ]
        x = lambda pose: pose[0][0]
        follower = lambda pose: pose[2]

        # list of poses sorted by the x coordinate (bigger to smaller)
        pose_by_x = sorted(pose_list, key=x, reverse=True)

        # choose follower1 and follower2 based on their y-coordinate
        if (pose_by_x[0][0][1] > pose_by_x[1][0][1]):
            pose_by_x[0][2] = 'follower1'
            pose_by_x[1][2] = 'follower2'
        else:
            pose_by_x[0][2] = 'follower2'
            pose_by_x[1][2] = 'follower1'
        # choose follower3 and follower4 based on their y-coordinate
        if (pose_by_x[2][0][1] > pose_by_x[3][0][1]):
            pose_by_x[2][2] = 'follower4'
            pose_by_x[3][2] = 'follower3'
        else:
            pose_by_x[2][2] = 'follower3'
            pose_by_x[3][2] = 'follower4'

        # list of poses sorted by follower (follower1 -> follower4)
        pose_by_follower = sorted(pose_by_x, key=follower)

        return pose_by_follower

    # Publishes velocities on the correct topics
    def publish_vel(self, turtle_id, follower_no):
        if turtle_id == 1:
            if follower_no == 1:
                self.cmd_turtle1_pub.publish(self.follower1)
            elif follower_no == 2:
                self.cmd_turtle1_pub.publish(self.follower2)
            elif follower_no == 3:
                self.cmd_turtle1_pub.publish(self.follower3)
            else:
                self.cmd_turtle1_pub.publish(self.follower4)
        elif turtle_id == 2:
            if follower_no == 1:
                self.cmd_turtle2_pub.publish(self.follower1)
            elif follower_no == 2:
                self.cmd_turtle2_pub.publish(self.follower2)
            elif follower_no == 3:
                self.cmd_turtle2_pub.publish(self.follower3)
            else:
                self.cmd_turtle2_pub.publish(self.follower4)
        elif turtle_id == 3:
            if follower_no == 1:
                self.cmd_turtle3_pub.publish(self.follower1)
            elif follower_no == 2:
                self.cmd_turtle3_pub.publish(self.follower2)
            elif follower_no == 3:
                self.cmd_turtle3_pub.publish(self.follower3)
            else:
                self.cmd_turtle3_pub.publish(self.follower4)
        else:
            if follower_no == 1:
                self.cmd_turtle4_pub.publish(self.follower1)
            elif follower_no == 2:
                self.cmd_turtle4_pub.publish(self.follower2)
            elif follower_no == 3:
                self.cmd_turtle4_pub.publish(self.follower3)
            else:
                self.cmd_turtle4_pub.publish(self.follower4)


    def execute_cb(self, goal):
        self.twist1 = Twist()
        self.twist2 = Twist()
        self.twist3 = Twist()
        self.twist4 = Twist()
        self.cmd_turtle1_pub = rospy.Publisher('/Turtle1/DD_controller/cmd_vel', Twist, queue_size=1)
        self.cmd_turtle2_pub = rospy.Publisher('/Turtle2/DD_controller/cmd_vel', Twist, queue_size=1)
        self.cmd_turtle3_pub = rospy.Publisher('/Turtle3/DD_controller/cmd_vel', Twist, queue_size=1)
        self.cmd_turtle4_pub = rospy.Publisher('/Turtle4/DD_controller/cmd_vel', Twist, queue_size=1)

        success = True

        '''
        # goal is organized in MODE, LENGTH and COUNT
        # mode    -> rendezvous     , circular traj
        # length  -> no_significance, radius[m]
        # count   -> no_significance, no_loop
        '''

        mode = goal.goal.x
        patrol_count = int(goal.goal.z)
        lined_up = False
        time_elapsed = 0

        pose_list = self.get_poses()
        for i in range(patrol_count):
            if mode == 1: ### RENDEZVOUS MODE ###
                ## Rendezvous to the origin (0,0)       
                while not (abs(self.compute_average_norm(pose_list)) < 0.5):       
                    pose_list = self.get_poses()
                    # Compute neighbour set for each agent:
                    N_1 = self.define_neighbours(pose_list[0], pose_list[1], pose_list[2], pose_list[3]) # list of neighbour of agent 1
                    N_2 = self.define_neighbours(pose_list[1], pose_list[0], pose_list[2], pose_list[3]) # list of neighbour of agent 2
                    N_3 = self.define_neighbours(pose_list[2], pose_list[0], pose_list[1], pose_list[3]) # list of neighbour of agent 3
                    N_4 = self.define_neighbours(pose_list[3], pose_list[0], pose_list[1], pose_list[2]) # list of neighbour of agent 4
                    # Compute gamma (the angular error)
                    gamma1 = posture_regulation.get_gamma(self.q1[0], self.q1[1], self.q1[2])
                    gamma2 = posture_regulation.get_gamma(self.q2[0], self.q2[1], self.q2[2])
                    gamma3 = posture_regulation.get_gamma(self.q3[0], self.q3[1], self.q3[2])
                    gamma4 = posture_regulation.get_gamma(self.q4[0], self.q4[1], self.q4[2])
                    # Compute inputs:
                    # for agent 1
                    if (len(N_1) != 0):
                        (v1, w1) = rendezvous_control_law(pose_list[0], N_1, gamma1)
                    else:
                        v1 = 0.0
                        w1 = 0.0        
                    # for agent 2
                    if (len(N_2) != 0):
                        (v2, w2) = rendezvous_control_law(pose_list[1], N_2, gamma2)
                    else:
                        v2 = 0.0
                        w2 = 0.0
                    # for agent 3
                    if (len(N_3) != 0):
                        (v3, w3) = rendezvous_control_law(pose_list[2], N_3, gamma3)
                    else:
                        v3 = 0.0
                        w3 = 0.0
                    # for agent 4
                    if (len(N_4) != 0):
                        (v4, w4) = rendezvous_control_law(pose_list[3], N_4, gamma4)
                    else:
                        v4 = 0.0
                        w4 = 0.0
                    # Publish velocities on cmd_vel_i topics
                    self.twist1.linear.x = -v1 + 0.2
                    self.twist1.angular.z = -w1
                    self.cmd_turtle1_pub.publish(self.twist1)
                    self.twist2.linear.x = -v2 + 0.2
                    self.twist2.angular.z = -w2
                    self.cmd_turtle2_pub.publish(self.twist2)
                    self.twist3.linear.x = -v3 + 0.2
                    self.twist3.angular.z = -w3
                    self.cmd_turtle3_pub.publish(self.twist3)
                    self.twist4.linear.x = -v4 + 0.2
                    self.twist4.angular.z = -w4
                    self.cmd_turtle4_pub.publish(self.twist4)
                    
                ## Rendezvous completed, now aligh to headind of 0rad
                start_time = time.time()
                while (lined_up == False ):
                    # reset linear velocities
                    self.twist1.linear.x = 0.0
                    self.twist2.linear.x = 0.0
                    self.twist3.linear.x = 0.0
                    self.twist4.linear.x = 0.0
                    # align robots
                    self.twist1.angular.z = align_cantrol_law(self.q1[2])
                    self.twist2.angular.z = align_cantrol_law(self.q2[2])
                    self.twist3.angular.z = align_cantrol_law(self.q3[2])
                    self.twist4.angular.z = align_cantrol_law(self.q4[2])
                    self.cmd_turtle1_pub.publish(self.twist1)
                    self.cmd_turtle2_pub.publish(self.twist2)
                    self.cmd_turtle3_pub.publish(self.twist3)
                    self.cmd_turtle4_pub.publish(self.twist4)
                    time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
                    if (time_elapsed > rospy.Duration.from_sec(12)):
                        lined_up = True


            elif mode == 2: ### CIRCLE MODE ###
                # abort if the group of agents is not at the origin
                pose_list = self.get_poses()                
                if not (abs(self.compute_average_norm(pose_list)) < 0.6):
                    self._result.result = "ABORTED"
                    self._as.set_aborted(self._result, "The rendezvous has not yet been carried out.")
                    return
                
                x_a, y_a = sp.symbols("x_a y_a")
                radius = goal.goal.y
                # defining the time vector
                period = 2*np.pi / (0.3/radius)
                self.t = np.linspace(0, period, 6000)
                max_t = self.t[len(self.t) - 1]
                len_t = len(self.t)
                # compute the desired trajectory
                w0 = 0.3/radius
                self.x_d =  radius*np.sin(self.t*w0)
                self.y_d = -radius*np.cos(self.t*w0) + radius
                self.th_d = self.t*w0
                self.xdot_d = radius*w0*np.cos(self.t*w0)
                self.ydot_d = radius*w0*np.sin(self.t*w0)


                rospy.sleep(0.1)  # sleep for 0.1s to setup self.q1,q2,q3,q4

                # weighted adjacency matrix
                A = np.array([ [0,1,1,1], [1,0,1,1], [1,1,0,1], [1,1,1,0] ])

                for i in np.arange(0, len(self.t)):

                    # get poses
                    pose0 = np.array([self.x_d[i], self.y_d[i], self.th_d[i]])  # "virtual leader"
                    pose1 = self.get_pose1()
                    pose2 = self.get_pose2()
                    pose3 = self.get_pose3()
                    pose4 = self.get_pose4()
                    # get the graph structure
                    sorted_by_follower = self.define_graph(pose1, pose2, pose3, pose4)

                    # get the apf function and inside_apf (True if inside the region where the repel force caused by the apf acts)
                    (V_a, inside_apf) = get_apf(sorted_by_follower[3][0][0], sorted_by_follower[3][0][1], self.x_obs, self.y_obs, self.r_obs, self.R_obs)

                    # defining th neighbour sets...
                    n_1 = [sorted_by_follower[1][0], sorted_by_follower[2][0], sorted_by_follower[3][0]]
                    n_2 = [sorted_by_follower[0][0], sorted_by_follower[2][0], sorted_by_follower[3][0]]
                    n_3 = [sorted_by_follower[0][0], sorted_by_follower[1][0], sorted_by_follower[3][0]]
                    n_4 = [sorted_by_follower[0][0], sorted_by_follower[1][0], sorted_by_follower[2][0]]
                    # ...and the desired geometric pattern
                    (pix_1, piy_1) = np.array([[self.p2x, self.p3x, self.p4x], [self.p2y, self.p3y, self.p4y]])
                    (pix_2, piy_2) = np.array([[self.p1x, self.p3x, self.p4x], [self.p1y, self.p3y, self.p4y]])
                    (pix_3, piy_3) = np.array([[self.p1x, self.p2x, self.p4x], [self.p1y, self.p2y, self.p4y]])
                    (pix_4, piy_4) = np.array([[self.p1x, self.p2x, self.p3x], [self.p1y, self.p2y, self.p3y]])

                    # get the control inputs for the followers from the controller based on the communication graph G
                    u11 = consensus_control_law1(pose0, w0, sorted_by_follower[0][0], n_1, A, 1)
                    u12 = consensus_control_law1(pose0, w0, sorted_by_follower[1][0], n_2, A, 2)
                    u13 = consensus_control_law1(pose0, w0, sorted_by_follower[2][0], n_3, A, 3)
                    u14 = consensus_control_law1(pose0, w0, sorted_by_follower[3][0], n_4, A, 4)
                    u1i_1 = np.array([u12, u13, u14])
                    u1i_2 = np.array([u11, u13, u14])
                    u1i_3 = np.array([u11, u12, u14])
                    u1i_4 = np.array([u11, u12, u13])
                    (v1, w1) = consensus_control_law2(pose0, w0, sorted_by_follower[0][0], self.p1x, self.p1y, n_1, pix_1, piy_1, A, 1, u11, u1i_1)
                    (v2, w2) = consensus_control_law2(pose0, w0, sorted_by_follower[1][0], self.p2x, self.p2y, n_2, pix_2, piy_2, A, 2, u12, u1i_2)
                    (v3, w3) = consensus_control_law2(pose0, w0, sorted_by_follower[2][0], self.p3x, self.p3y, n_3, pix_3, piy_3, A, 3, u13, u1i_3)
                    (v4, w4) = consensus_control_law2(pose0, w0, sorted_by_follower[3][0], self.p4x, self.p4y, n_4, pix_4, piy_4, A, 4, u14, u1i_4)

                    # check wheter we are in the presence of an obstacle
                    if not inside_apf:
                        self.follower1.linear.x = 0.01*v1
                        self.follower2.linear.x = 0.01*v2
                        self.follower3.linear.x = 0.01*v3
                        self.follower4.linear.x = 0.01*v4
                    else:
                        # we are in proximity of an obstacle -> different control law
                        # get modified position tracking error
                        (err_sym_x, err_sym_y) = get_modified_position_error(sorted_by_follower[3][0][0], sorted_by_follower[3][0][1], self.x_d[i], self.y_d[i], V_a)
                        err_x = err_sym_x.subs(x_a, sorted_by_follower[3][0][0]).subs(y_a, sorted_by_follower[3][0][1])
                        err_y = err_sym_y.subs(x_a, sorted_by_follower[3][0][0]).subs(y_a, sorted_by_follower[3][0][1])
                        # get control inputs for obstacle avoidance phase
                        v = obstacle_control_law(err_x, err_y, sorted_by_follower[0][0][2], self.xdot_d[i], self.ydot_d[i])
                        self.follower1.linear.x = v 
                        self.follower2.linear.x = v 
                        self.follower3.linear.x = v 
                        self.follower4.linear.x = v                    
                    
                    self.follower1.angular.z = -0.01*w1
                    self.follower2.angular.z = -0.01*w2
                    self.follower3.angular.z = -0.01*w3
                    self.follower4.angular.z = -0.01*w4

                    # publish velocities on /cmd_vel topics
                    self.publish_vel(sorted_by_follower[0][1], 1)
                    self.publish_vel(sorted_by_follower[1][1], 2)
                    self.publish_vel(sorted_by_follower[2][1], 3)
                    self.publish_vel(sorted_by_follower[3][1], 4)

                    rospy.sleep(max_t/len_t)


                # stop all the robots
                self.follower1.linear.x = 0.0
                self.follower1.angular.z = 0.0
                self.cmd_turtle1_pub.publish(self.follower1)
                self.cmd_turtle2_pub.publish(self.follower1)
                self.cmd_turtle3_pub.publish(self.follower1)
                self.cmd_turtle4_pub.publish(self.follower1)


        if success:
            self._result.result = "SUCCEDED"
            rospy.loginfo('%s: succeeded' % self._action_name)
            self._as.set_succeeded(self._result, "Operation completed successfully!")
        


if __name__ == '__main__':
    rospy.init_node('group')
    server = GroupAction(rospy.get_name())
    rospy.spin()
    