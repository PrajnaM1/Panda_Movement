#! /usr/bin/env python

import rospy
import actionlib
from demo.msg import robot_geoAction, robot_geoFeedback, robot_geoResult
import math
import tf
import PyKDL 
import tf2_ros
import geometry_msgs.msg
from tf.transformations import *
import numpy as np
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import String
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class action_server():

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer("arm_as", robot_geoAction, execute_cb = self.execute_cb, auto_start = False)
        self.a_server.start()

    def execute_cb(self, goal):

        success = True

        #Status of Movement Completion
        move_status = False
        
        feedback = robot_geoFeedback()
    
        result = robot_geoResult()
        completion_status_lin = False
        completion_status_ang = False

        #Listen to transforms
        listener = tf.TransformListener()
        rospy.sleep(1)

        rate = rospy.Rate(100.0) 

        pose_msg = geometry_msgs.msg.Pose()
        
        #current_pose from tf
        (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
        current_pose = [round(trans[0],2), round(trans[1],2), round(trans[2],2), round(rot[0],2), round(rot[1],2), round(rot[2],2), round(rot[3],2)]
        print(current_pose)
        #current_pose = [round(ee2b.transform.translation.x,2), round(ee2b.transform.translation.y,2), round(ee2b.transform.translation.z,2), round(ee2b.transform.rotation.x,2), round(ee2b.transform.rotation.y,2), round(ee2b.transform.rotation.z,2), round(ee2b.transform.rotation.w,2)]

        #goal_pose from client
        goal_pose = [round(goal.coord.pose.position.x,2), round(goal.coord.pose.position.y,2), round(goal.coord.pose.position.z,2), round(goal.coord.pose.orientation.x,2), round(goal.coord.pose.orientation.y,2), round(goal.coord.pose.orientation.z,2), round(goal.coord.pose.orientation.w,2)]
        
        #ACTION: MOVE TO GOAL/CONTACT (LINEAR)
        #We know where the goal is and where the robot arm is. Interpolate path between these 2 points and move along these points. 
        if (goal.type == 'move to contact'):
            #while(not rospy.is_shutdown):
            x_diff = round((goal_pose[0] - current_pose[0]),2)
            y_diff = round((goal_pose[1] - current_pose[1]),2)
            z_diff = round((goal_pose[2] - current_pose[2]),2)

            #while ((round(current_pose[0],2) != round(goal_pose[0],2))  or (round(current_pose[1],2) != round(goal_pose[1],2)) or (round(current_pose[2],2) != round(goal_pose[2],2)) ):
            while ( (abs(x_diff) > 0.01)  or (abs(y_diff) > 0.01) or (abs(z_diff) > 0.01) ):
                
                if self.a_server.is_preempt_requested():
                    success = False
                    break 

                #Interpolation between current_pose and goal_pose to get the set of waypoints
                max_step_size = 0.01
                x_diff = round((goal_pose[0] - current_pose[0]),2)
                y_diff = round((goal_pose[1] - current_pose[1]),2)
                z_diff = round((goal_pose[2] - current_pose[2]),2)

                print("x_diff is:", x_diff)
                print("y_diff is:", y_diff)
                print("z_diff is:", z_diff)

                #If movement is NOT required along all directions
                if abs(x_diff) <= 0.01:

                    if abs(y_diff) <= 0.01:
                        print("x_diff and y_diff are zero")
                        if current_pose[2] < goal_pose[2]:
                            z = np.arange(current_pose[2], goal_pose[2], 0.01)
                        else:
                            z = np.arange(current_pose[2], goal_pose[2], -0.01)
                        num_pts = len(z)
                        x = np.full(num_pts, current_pose[0])
                        y = np.full(num_pts, current_pose[1])    

                    elif abs(z_diff) <= 0.01:
                        print("x_diff and z_diff are zero")
                        if current_pose[1] < goal_pose[1]:
                            y = np.arange(current_pose[1], goal_pose[1], 0.01)
                        else:
                            y = np.arange(current_pose[1], goal_pose[1], -0.01)
                        num_pts = len(y)
                        x = np.full(num_pts, current_pose[0])
                        z = np.full(num_pts, current_pose[2])
                        

                    else:
                        print("x_diff is zero")
                        if abs(y_diff) > abs(z_diff):
                            print("y_diff is greatest in magnitude")
                            if current_pose[1] < goal_pose[1]:
                                y = np.arange(current_pose[1], goal_pose[1], 0.01)
                            else:
                                y = np.arange(current_pose[1], goal_pose[1], -0.01)
                            num_pts = len(y)
                            z_step = z_diff/num_pts
                            z = np.arange(current_pose[2], goal_pose[2], z_step)
                        else:
                            print("z_diff is greatest in magnitude")
                            if current_pose[2] < goal_pose[2]:
                                z = np.arange(current_pose[2], goal_pose[2], 0.01)
                            else:
                                z = np.arange(current_pose[2], goal_pose[2], -0.01)
                            num_pts = len(z)
                            print("Number of z points:", num_pts)
                            y_step = y_diff/num_pts
                            y = np.arange(current_pose[1], goal_pose[1], y_step)
                            print("Number of y points:", len(y))
                        #x = np.array((1, num_pts))
                        val = current_pose[0]
                        x = np.full(num_pts, val)
                        #print("Length of x with same number of points is:", len(x))

                elif abs(y_diff) <= 0.01:

                    if abs(x_diff) <= 0.01:
                        print("x_diff and y_diff are zero")
                        if current_pose[2] < goal_pose[2]:
                            z = np.arange(current_pose[2], goal_pose[2], 0.01)
                        else:
                            z = np.arange(current_pose[2], goal_pose[2], -0.01)
                        num_pts = len(z)
                        x = np.full(num_pts, current_pose[0])
                        y = np.full(num_pts, current_pose[1])
                        

                    elif abs(z_diff) <= 0.01:
                        print("y_diff and z_diff are zero")
                        if current_pose[0] < goal_pose[0]:
                            x = np.arange(current_pose[0], goal_pose[0], 0.01)
                        else:
                            x = np.arange(current_pose[0], goal_pose[0], -0.01)
                        num_pts = len(x)
                        y = np.full(num_pts, current_pose[1])
                        z = np.full(num_pts, current_pose[2])
                        
                    else:
                        print("y_diff is zero")
                        if abs(x_diff) > abs(z_diff):
                            print("x_diff is greatest in magnitude")
                            if current_pose[0] < goal_pose[0]:
                                x = np.arange(current_pose[0], goal_pose[0], 0.01)
                            else:
                                x = np.arange(current_pose[0], goal_pose[0], -0.01)
                            num_pts = len(x)
                            z_step = z_diff/num_pts
                            z = np.arange(current_pose[2], goal_pose[2], z_step)
                        else:
                            print("z_diff is greatest in magnitude")
                            if current_pose[2] < goal_pose[2]:
                                z = np.arange(current_pose[2], goal_pose[2], 0.01)
                            else:
                                z = np.arange(current_pose[2], goal_pose[2], -0.01)
                            num_pts = len(z)
                            x_step = x_diff/num_pts
                            x = np.arange(current_pose[0], goal_pose[0], x_step)
                        y = np.full(num_pts, current_pose[1])

                elif abs(z_diff) <= 0.01:

                    if abs(x_diff) <= 0.01:
                        print("x_diff and z_diff are zero")
                        if current_pose[1] < goal_pose[1]:
                            y = np.arange(current_pose[1], goal_pose[1], 0.01)
                        else:
                            y = np.arange(current_pose[1], goal_pose[1], -0.01)
                        num_pts = len(y)
                        x = np.full(num_pts, current_pose[0])
                        z = np.full(num_pts, current_pose[2])
                        print("Number of x points:", len(x))
                        print("Number of y points:", len(y))
                        print("Number of z points:", len(z))
                        

                    elif abs(y_diff) <= 0.01:
                        print("y_diff and z_diff are zero")
                        if current_pose[0] < goal_pose[0]:
                            x = np.arange(current_pose[0], goal_pose[0], 0.01)
                        else:
                            x = np.arange(current_pose[0], goal_pose[0], -0.01)
                        num_pts = len(x)
                        y = np.full(num_pts, current_pose[1])
                        z = np.full(num_pts, current_pose[2])
                        

                    else:
                        print("z_diff is zero")
                        if abs(x_diff) > abs(y_diff):
                            print("x_diff is greatest in magnitude")
                            if current_pose[0] < goal_pose[0]:
                                x = np.arange(current_pose[0], goal_pose[0], 0.01)
                            else:
                                x = np.arange(current_pose[0], goal_pose[0], -0.01)
                            num_pts = len(x)
                            y_step = y_diff/num_pts
                            y = np.arange(current_pose[1], goal_pose[1], y_step)
                        else:
                            print("y_diff is greatest in magnitude")
                            if current_pose[1] < goal_pose[1]:
                                y = np.arange(current_pose[1], goal_pose[1], 0.01)
                            else:
                                y = np.arange(current_pose[1], goal_pose[1], -0.01)
                            num_pts = len(y)
                            x_step = x_diff/num_pts
                            x = np.arange(current_pose[0], goal_pose[0], x_step)
                        z = np.full(num_pts, current_pose[2])

                #If movement is required along all directions
                else:
                    
                    if (abs(x_diff) >= abs(y_diff)) and (abs(x_diff) >= abs(z_diff)):
                        max_diff = x_diff
                        print("max_diff in magnitude is:", max_diff)
                        print("x_diff is greatest")
                        if current_pose[0] < goal_pose[0]:
                            x = np.arange(current_pose[0], goal_pose[0], 0.01)
                        else:
                            x = np.arange(current_pose[0], goal_pose[0], -0.01)
                        num_pts = len(x)
                        y_step = y_diff/num_pts
                        y = np.arange(current_pose[1], goal_pose[1], y_step)
                        z_step = z_diff/num_pts
                        z = np.arange(current_pose[2], goal_pose[2], z_step)
                        print(x)
                        print(y)
                        print(z)

                    elif (abs(y_diff) >= abs(x_diff)) and (abs(y_diff) >= abs(z_diff)):
                        max_diff = y_diff
                        print("max_diff in magnitude is:", max_diff)
                        print("y_diff is greatest")
                        if current_pose[1] < goal_pose[1]:
                            y = np.arange(current_pose[1], goal_pose[1], 0.01)
                        else:
                            y = np.arange(current_pose[1], goal_pose[1], -0.01)
                        num_pts = len(y)
                        x_step = x_diff/num_pts
                        x = np.arange(current_pose[0], goal_pose[0], x_step)
                        z_step = z_diff/num_pts
                        z = np.arange(current_pose[2], goal_pose[2], z_step)
                        print(x)
                        print(y)
                        print(z)

                    elif (abs(z_diff) >= abs(x_diff)) and (abs(z_diff) >= abs(y_diff)):
                        max_diff = z_diff
                        print("max_diff in magnitude is:", max_diff)
                        print("z_diff is greatest")
                        if current_pose[2] < goal_pose[2]:
                            z = np.arange(current_pose[2], goal_pose[2], 0.01)
                        else:
                            z = np.arange(current_pose[2], goal_pose[2], -0.01)
                        num_pts = len(z)
                        x_step = x_diff/num_pts
                        x = np.arange(current_pose[0], goal_pose[0], x_step)
                        y_step = y_diff/num_pts
                        y = np.arange(current_pose[1], goal_pose[1], y_step)
                        print(x)
                        print(y)
                        print(z)
                    '''
                    elif (abs(x_diff)==abs(y_diff)):
                        if (abs(z_diff) > abs(x_diff)):
                            max_diff = z_diff
                            print("max_diff in magnitude is:", max_diff)
                            print("z_diff is greatest")
                            z = np.arange(current_pose[2], goal_pose[2], 0.01)
                            num_pts = len(z)
                            x_step = x_diff/num_pts
                            x = np.arange(current_pose[0], goal_pose[0], x_step)
                            y_step = y_diff/num_pts
                            y = np.arange(current_pose[1], goal_pose[1], y_step)
                            print(x)
                            print(y)
                            print(z)

                        
                        
                    elif (abs(x_diff)==abs(z_diff)):

                    elif (abs(y_diff)==abs(z_diff)):
                    '''
                    
                    '''
                    x = np.arange(current_pose[0], goal_pose[0], 0.05)
                    y = np.arange(current_pose[1], goal_pose[1], 0.05)
                    z = np.arange(current_pose[2], goal_pose[2], 0.05)
                    '''
                    '''
                    f = interpolate.interp2d(x,y,z, kind='linear')

                    xnew = np.arange(current_pose[0], goal_pose[0], 0.05)
                    ynew = np.arange(current_pose[1], goal_pose[1], 0.05)
                    znew = f(xnew,ynew)
                    '''
                #Make sure the number of elements in x, y and z are the same
                if len(y) > len(z):
                    diff_ele = -len(z) + len(y)
                    for i in range(diff_ele):
                        y = np.delete(y,i+1)
                
                    print("Number of z points after shaping:", len(z))  
                    print("Number of y points after shaping:", len(y)) 


                #Array of point    
                pts = np.concatenate((x,y,z))
                print("Shape of pts array:", pts.shape)
                print("Total number of points:", len(pts))

                for idx in range(num_pts-1):
                    
                    #Publish the point to ee_path_goals
                    pose_msg.position.x = pts[idx]
                    pose_msg.position.y = pts[idx + num_pts + 1]
                    pose_msg.position.z = pts[idx + (2*num_pts) + 1]
                    pose_msg.orientation.x = 0.0
                    pose_msg.orientation.y = 0.0
                    pose_msg.orientation.z = 0.0
                    pose_msg.orientation.w = 1.0

                    pose_pub.publish(pose_msg)
                    time.sleep(1)
                    print("Moved to point: ", pose_msg.position.x, pose_msg.position.y, pose_msg.position.z)
                    time.sleep(5)

                time.sleep(5) 
                (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time())
                    
                current_pose = [round(trans[0],2), round(trans[1],2), round(trans[2],2), round(rot[0],2), round(rot[1],2), round(rot[2],2), round(rot[3],2)]
                print("Current pose is:", trans, rot)
                #current_pose = [round(ee2b.transform.translation.x,2), round(ee2b.transform.translation.y,2), round(ee2b.transform.translation.z,2), round(ee2b.transform.rotation.x,2), round(ee2b.transform.rotation.y,2), round(ee2b.transform.rotation.z,2), round(ee2b.transform.rotation.w,2)]

            move_status = True

            feedback.movement = 'Movement Complete!'
            self.a_server.publish_feedback(feedback)
            time.sleep(1)

            #Result = final end_effector coordinates
            result.final_coord.pose.position.x = trans[0]
            result.final_coord.pose.position.y = trans[1]
            result.final_coord.pose.position.z = trans[2]
            result.final_coord.pose.orientation.x = rot[0]
            result.final_coord.pose.orientation.y = rot[1]
            result.final_coord.pose.orientation.z = rot[2]
            result.final_coord.pose.orientation.w = rot[3]
            self.a_server.set_succeeded(result)
        
            rate.sleep() 

if __name__=="__main__":

    rospy.init_node('action_server')

    pose_pub = rospy.Publisher('/panda/ee_path_goals', geometry_msgs.msg.Pose, queue_size=1)
 
    s = action_server()

    rospy.spin()

