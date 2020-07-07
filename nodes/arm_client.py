#! /usr/bin/env python

import rospy 
import actionlib
from demo.msg import robot_geoAction, robot_geoGoal

#In this program, goal coordinates and type of action are specified and sent to the server for arm movement

def feedback_cb(msg):
    print('Feedback received: ', msg)

def call_server():

    client = actionlib.SimpleActionClient('arm_as', robot_geoAction)
    client.wait_for_server()

    goal = robot_geoGoal()

    #SPECIFY GOAL, ACTION & DIRECTION(only for screw)
    '''
    goal.coord[0] = 0.6
    goal.coord[1] = 0.1
    goal.coord[2] = 0.6
    goal.coord[3] = -0.3
    goal.coord[4] = 0.0
    goal.coord[5] = 0.0
    goal.coord[6] = 1.0
    '''
    goal.coord.pose.position.x = 0.55
    goal.coord.pose.position.y = 0.0
    goal.coord.pose.position.z = 0.0
    goal.coord.pose.orientation.x = 0.0
    goal.coord.pose.orientation.y = 0.0
    goal.coord.pose.orientation.z = 0.0
    goal.coord.pose.orientation.w = 1.0

    goal.type = 'wipe'
    #Selection Vector
    goal.selection.x = 1.0
    goal.selection.y = 1.0
    goal.selection.z = 0.0

    #Wrench
    goal.ee_wrench.force.x = 0.0
    goal.ee_wrench.force.y = 0.0
    goal.ee_wrench.force.z = -5.0
    goal.ee_wrench.torque.x = 0.0
    goal.ee_wrench.torque.y = 0.0
    goal.ee_wrench.torque.z = 0.0

    #goal.direction = 'CLOCKWISE'
    #goal.obj_reach = True
    #goal.gripper_content = False
    #goal.screwable = True

    client.send_goal(goal, feedback_cb=feedback_cb)
     
    client.wait_for_result()
    result = client.get_result()

    return result                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      


if __name__=="__main__":

    try:
        rospy.init_node('action_client')

        result = call_server()                                             

        print('The end effector is at position: ', result.final_coord.pose.position.x, result.final_coord.pose.position.y, result.final_coord.pose.position.z, result.final_coord.pose.orientation.x, result.final_coord.pose.orientation.y, result.final_coord.pose.orientation.z, result.final_coord.pose.orientation.w)

    except rospy.ROSInterruptException as e:
        print('Exception occured!', e)






