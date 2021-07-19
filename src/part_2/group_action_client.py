#!/usr/bin/env python3

import rospy
import actionlib
from mr_project.msg import MoveGroupAction, MoveGroupGoal, MoveGroupResult, MoveGroupFeedback
import sys

msg = """

Move the agents in formation!
-----------------------
mode : r - Rendezvous to the origin
       c - Follow a Circular trajectory

length : Rendezvoud mode - not used
         Circle mode - radius of the circle (m)

count : lap count (if Circle mode)

If you want to close, insert 'x'

"""

class Client():
    def __init__(self):
        rospy.loginfo("Waiting for server")
        self.client()

    def getkey(self):
        mode, length, count = input("| mode | length | count |\n").split()
        mode, length, count = [str(mode), float(length), int(count)]
        if mode == 'r':
            mode = 1
        elif mode == 'c':
            mode = 2
        elif mode == 'x':
            self.shutdown()
        else:
            rospy.loginfo("Invalid mode selected")
        return mode, length, count

    def client(self):
        client = actionlib.SimpleActionClient('group', MoveGroupAction)
        mode, length, count = self.getkey()
        client.wait_for_server()
        goal = MoveGroupGoal()
        goal.goal.x = mode
        goal.goal.y = length
        goal.goal.z = count
        client.send_goal(goal)
        rospy.loginfo("sendig goal to server...")
        client.wait_for_result()

        rospy.loginfo(client.get_result())
        rospy.loginfo('[Result] status: %s' %(client.get_goal_status_text()))


    def shutdown(self):
        rospy.sleep(1)        


if __name__ == '__main__':
    rospy.init_node('group_client', anonymous=True)
    try:
        while not rospy.is_shutdown():
            print(msg)
            result = Client()
    except:
        print("Program close.", file=sys.stderr)
