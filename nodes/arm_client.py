#! /usr/bin/env python

import rospy 
import actionlib
from demo.msg import robotAction, robotGoal

#In this program, goal coordinates and type of action are specified and sent to the server for arm movement

def feedback_cb(msg):
    print('Feedback received: ', msg)

def call_server():

    client = actionlib.SimpleActionClient('arm_as', robotAction)
    client.wait_for_server()

    goal = robotGoal()

    #SPECIFY GOAL AND ACTION
    goal.coord[0] = 0.5
    goal.coord[1] = 0.0
    goal.coord[2] = 0.5
    goal.coord[3] = 0.3
    goal.coord[4] = 0.0
    goal.coord[5] = 0.0
    goal.coord[6] = 1.0
    goal.type = 'PLACE'

    client.send_goal(goal, feedback_cb=feedback_cb)
     
    client.wait_for_result()
    result = client.get_result()

    return result                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      


if __name__=="__main__":

    try:
        rospy.init_node('action_client')

        result = call_server()                                             

        print('The end effector is at position: ', result)

    except rospy.ROSInterruptException as e:
        print('Exception occured!', e)






