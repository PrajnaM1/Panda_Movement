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
from numpy.linalg import norm
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import String
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class action_server():

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer("arm_as", robot_geoAction, execute_cb = self.execute_cb, auto_start = False)
        self._force_sub = rospy.Subscriber("/panda/wrench", geometry_msgs.msg.Wrench, self.move, queue_size = 1)
        
        self.a_server.start()       

    def execute_cb(self, goal):

        self.action = goal.type

        self.goal_wrench_f_x = goal.ee_wrench.force.x
        self.goal_wrench_f_y = goal.ee_wrench.force.y
        self.goal_wrench_f_z = goal.ee_wrench.force.z
        self.goal_wrench_t_x = goal.ee_wrench.torque.x
        self.goal_wrench_t_y = goal.ee_wrench.torque.y
        self.goal_wrench_t_z = goal.ee_wrench.torque.z
    
        self.sel_x = goal.selection.x 
        self.sel_y = goal.selection.y 
        self.sel_z = goal.selection.z 

        self.feedback = robot_geoFeedback()
        self.result = robot_geoResult()

        #Listen to transforms
        listener = tf.TransformListener()
        rospy.sleep(1)
        rate = rospy.Rate(100.0) 

        #current_pose from tf
        (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time(0))
        self.current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
        print(current_pose)

        #goal_pose from client
        self.goal_pose = [round(goal.coord.pose.position.x,3), round(goal.coord.pose.position.y,3), round(goal.coord.pose.position.z,3), round(goal.coord.pose.orientation.x,3), round(goal.coord.pose.orientation.y,3), round(goal.coord.pose.orientation.z,3), round(goal.coord.pose.orientation.w,3)]
        
    def move(self, msg):

        success = True

        #Status of Movement Completion
        move_status = False

        force_x = msg.force.x
        force_y = msg.force.x
        force_z = msg.force.x

        pose_msg = geometry_msgs.msg.Pose()
        selection_msg = geometry_msgs.msg.Vector3()
        wrench_msg = geometry_msgs.msg.Wrench()

        force_r = round(np.sqrt(force_x**2 + force_y**2 + force_z**2), 2)
        
        #We know where the goal is and where the robot arm is. Interpolate path between these 2 points and move along these points. 
        if (self.action == 'move to contact'):
            
            x_diff = round((self.goal_pose[0] - self.current_pose[0]),3)
            y_diff = round((self.goal_pose[1] - self.current_pose[1]),3)
            z_diff = round((self.goal_pose[2] - self.current_pose[2]),3)

            x_ang_diff = round((self.goal_pose[3] - self.current_pose[3]),3)
            y_ang_diff = round((self.goal_pose[4] - self.current_pose[4]),3)
            z_ang_diff = round((self.goal_pose[5] - self.current_pose[5]),3)
            w_ang_diff = round((self.goal_pose[6] - self.current_pose[6]),3)

            while ( (abs(x_diff) > 0.015)  or (abs(y_diff) > 0.015) or (abs(z_diff) > 0.015) or abs(force_r > 18.00) ):

                if self.a_server.is_preempt_requested():
                    success = False
                    break 

                #LINEAR
                r = np.sqrt(x_diff**2 + y_diff**2 + z_diff**2)
                
                if abs(r) >= 0.1:
                    step_size = 0.05
                else:
                    step_size = 0.01

                num_pts = abs(int((r // step_size) + 1))

                if (abs(x_diff) > 0.015):
                    x_step = x_diff / num_pts
                    x = np.arange(self.current_pose[0], self.goal_pose[0], x_step)
                else: 
                    print("x_diff nearly 0")
                    x = np.full(num_pts, self.current_pose[0])

                if (abs(y_diff) > 0.015):
                    y_step = y_diff / num_pts
                    y = np.arange(self.current_pose[1], self.goal_pose[1], y_step)
                else: 
                    print("y_diff nearly 0")
                    y = np.full(num_pts, self.current_pose[1])

                if (abs(z_diff) > 0.015):
                    z_step = z_diff / num_pts
                    z = np.arange(self.current_pose[2], self.goal_pose[2], z_step)
                else: 
                    print("z_diff nearly 0")
                    z = np.full(num_pts, self.current_pose[2])
                
                #ANGULAR
                key_rots = R.from_quat([ [self.current_pose[3], self.current_pose[4], self.current_pose[5], self.current_pose[6]], [self.goal_pose[3], self.goal_pose[4], self.goal_pose[5], self.goal_pose[6]]])
                key_times = [0, 1]

                slerp = Slerp(key_times, key_rots)

                times = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
                interp_rots = slerp(times)

                quat = interp_rots.as_quat()

                #Publish the point to ee_path_goals
                pose_msg.position.x = round (x[1], 3)
                pose_msg.position.y = round (y[1], 3)
                pose_msg.position.z = round (z[1], 3)
                pose_msg.orientation.x = round(quat[6][0], 3)
                pose_msg.orientation.y = round(quat[6][1], 3)
                pose_msg.orientation.z = round(quat[6][2], 3)
                pose_msg.orientation.w = round(quat[6][3], 3)

                pose_pub.publish(pose_msg)
                print("Moving to point: ", pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w )
                time.sleep(5)

                (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time())
                self.current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
                print("Current pose is:", self.current_pose)

                x_diff = round((self.goal_pose[0] - self.current_pose[0]),3)
                y_diff = round((self.goal_pose[1] - self.current_pose[1]),3)
                z_diff = round((self.goal_pose[2] - self.current_pose[2]),3)

                x_ang_diff = round((self.goal_pose[3] - self.current_pose[3]),3)
                y_ang_diff = round((self.goal_pose[4] - self.current_pose[4]),3)
                z_ang_diff = round((self.goal_pose[5] - self.current_pose[5]),3)
                w_ang_diff = round((self.goal_pose[6] - self.current_pose[6]),3)

                
            move_status = True
            print('Movement Complete!')
            time.sleep(1)

            if (self.action == "wipe"):
               
                #Publish Selection Vector
                selection_msg.x = self.sel_x 
                selection_msg.y = self.sel_y 
                selection_msg.z = self.sel_z

                selection_pub.publish(selection_msg)

                #Publish Wrench
                wrench_msg.force.x = self.goal_wrench_f_x
                wrench_msg.force.y = self.goal_wrench_f_y
                wrench_msg.force.z = self.goal_wrench_f_z
                wrench_msg.torque.x = self.goal_wrench_t_x
                wrench_msg.torque.y = self.goal_wrench_t_y
                wrench_msg.torque.z = self.goal_wrench_t_z
                wrench_pub.publish(wrench_msg)

                #Publish the movement
                (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time())
                self.current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
                time.sleep(0.5)

                pose_msg.position.x = self.current_pose[0]
                pose_msg.position.y = self.current_pose[1] + 0.2
                pose_msg.position.z = self.current_pose[2]
                pose_msg.orientation.x = self.current_pose[3]
                pose_msg.orientation.y = self.current_pose[4]
                pose_msg.orientation.z = self.current_pose[5]
                pose_msg.orientation.w = self.current_pose[6]
                pose_pub.publish(pose_msg)
                time.sleep(5)

                (trans,rot) = listener.lookupTransform('panda_link0', 'end_effector', rospy.Time())
                self.current_pose = [round(trans[0],3), round(trans[1],3), round(trans[2],3), round(rot[0],3), round(rot[1],3), round(rot[2],3), round(rot[3],3)]
                time.sleep(0.5)

                pose_msg.position.x = self.current_pose[0]
                pose_msg.position.y = self.current_pose[1] - 0.2
                pose_msg.position.z = self.current_pose[2]
                pose_msg.orientation.x = self.current_pose[3]
                pose_msg.orientation.y = self.current_pose[4]
                pose_msg.orientation.z = self.current_pose[5]
                pose_msg.orientation.w = self.current_pose[6]
                pose_pub.publish(pose_msg)

                #Stop force application
                wrench_msg.force.x = 0.0
                wrench_msg.force.y = 0.0
                wrench_msg.force.z = 0.0
                wrench_msg.torque.x = 0.0
                wrench_msg.torque.y = 0.0
                wrench_msg.torque.z = 0.0
                wrench_pub.publish(wrench_msg)

                
            print('Wiping Complete!')

            feedback.movement = 'Movement and Wiping Complete!'
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
    selection_pub = rospy.Publisher('/panda/selection', geometry_msgs.msg.Vector3, queue_size=1)
    #wrench_pub = rospy.Publisher('/panda/ee_wrench_goals', geometry_msgs.msg.Wrench, queue_size=1)
    
    s = action_server()

    rospy.spin()

