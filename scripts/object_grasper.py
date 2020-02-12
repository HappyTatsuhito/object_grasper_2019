#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import time
import math
import threading
import actionlib
# -- ros msgs --
from std_msgs.msg import Bool,Float64,String
from dynamixel_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
#from sensor_msgs.msg import LaserScan
# --  ros srvs --
from manipulation.srv import ManipulateSrv
# -- action msgs --
from manipulation.msg import *
# -- class inheritance --
from motor_controller import ArmPoseChanger

class ObjectGrasper(ArmPoseChanger):
    def __init__(self):
        super(ObjectGrasper,self).__init__()
        # -- topic subscriber --
        navigation_place_sub = rospy.Subscriber('/navigation/move_place',String,self.navigationPlaceCB)
        # -- topic publisher --
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size = 1)
        # -- action server --
        self.act = actionlib.SimpleActionServer('/object/grasp',
                                                ObjectGrasperAction,
                                                execute_cb = self.actionMain,
                                                auto_start = False)
        self.act.register_preempt_callback(self.actionPreempt)
        # -- instance variables --
        self.navigation_place = 'Null'
        self.target_place = {'Null':0.75, 'Eins':0.72, 'Zwei':0.66, 'Drei':0.69}
        #self.front_laser_dist = 0.00

        self.act.start()

    def placeMode(self):
        self.moveBase(-0.55)
        y = self.target_place[self.navigation_place] + 0.1
        x = (y-0.75)/10+0.5
        joint_angle = self.inverseKinematics(x, y)
        if not joint_angle:
            return
        self. wristPub(joint_angle[2])
        while self.m3_is_moving and not rospy.is_shutdown():
            pass
        rospy.sleep(0.1)
        m3_angle = self.m3_current_pos
        m3_diff = self.m3_error
        shoulder_param = -1*(joint_angle[1]/2.1+m3_angle)*2.1-(m3_diff+0.01)*32
        # -(m3_diff+0.025)*32：重さ補正
        rospy.sleep(0.2)
        self.shoulderPub(shoulder_param)
        self.elbowPub(joint_angle[1])
        self.moveBase(0.6)
        rospy.sleep(0.4)
        self.armController(shoulder_param, joint_angle[1]-0.6, m3_angle+0.3)
        rospy.sleep(0.2)
        self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
        rospy.sleep(0.5)
        self.moveBase(-0.3)
        self.shoulderPub(shoulder_param+0.2)
        self.moveBase(-0.8)
        self.changeArmPose('carry')
        self.navigation_place = 'Null'
        rospy.loginfo('  Finish place command\n')
        return True
                        
    def moveBase(self,rad_speed):
        cmd = Twist()
        for speed_i in range(10):
            cmd.linear.x = rad_speed*0.05*speed_i
            cmd.angular.z = 0
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(0.1)
        for speed_i in range(10):
            cmd.linear.x = rad_speed*0.05*(10-speed_i)
            cmd.angular.z = 0
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(0.1)
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.cmd_vel_pub.publish(cmd)

    def approachObject(self,object_centroid):
        if object_centroid.x < 0.6 or object_centroid.x > 0.75:
            move_range = (object_centroid.x-0.7)*2.0
            if abs(move_range) < 0.4:
                move_range = int(move_range/abs(move_range))*0.4
            self.moveBase(move_range)
            return False
        else :
            return True

    def graspObject(self, object_centroid):
        rospy.loginfo('\n----- Grasp Object -----')
        self.moveBase(-0.5)
        if self.navigation_place == 'Null':
            y = object_centroid.z + 0.06
        else:
            y = self.target_place[self.navigation_place] + 0.13
        x = (y-0.75)/10+0.5 #0.5
        joint_angle = self.inverseKinematics(x, y)
        if not joint_angle:
            return False
        self.armController(joint_angle[0], joint_angle[1], joint_angle[2])
        rospy.sleep(2.5)
        move_range = (0.17+object_centroid.x+0.15-(x+0.2))*4.5
        self.moveBase(move_range*0.7)
        rospy.sleep(0.3)
        self.moveBase(move_range*0.4)
        grasp_flg = self.endeffectorPub(True)
        rospy.sleep(1.0)
        self.shoulderPub(joint_angle[0]+0.2)
        self.moveBase(-0.6)
        #self.shoulderPub(0.7) # 重い物体を把持した場合に必要
        self.changeArmPose('carry')
        rospy.sleep(4.0)
        if grasp_flg :
            grasp_flg = self.m4_error > 0.03
        if grasp_flg :
            rospy.loginfo('Successfully grasped the object!')
        else:
            self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
            rospy.loginfo('Failed to grasp the object.')
        rospy.loginfo('Finish grasp.')
        return grasp_flg
    
    def navigationPlaceCB(self,res):
        target_dic = {'Eins':['desk'], 'Zwei':['cupboard'], 'Drei':['shelf']}
        for key,value in target_dic.items():
            if res.data in value:
                self.navigation_place = key
                break
            else:
                self.navigation_place = 'Null'

    def actionPreempt(self):
        rospy.loginfo('Preempt callback')
        self.act.set_preempted(text = 'message for preempt')
        self.preempt_flg = True

    def actionMain(self,object_centroid):
        target_centroid = object_centroid.grasp_goal
        #grasp_feedback = ObjectGrasperFeedback()
        grasp_result = ObjectGrasperResult()
        grasp_flg = False
        approach_flg = self.approachObject(target_centroid)
        if approach_flg:
            grasp_flg = self.graspObject(target_centroid)
        grasp_result.grasp_result = grasp_flg
        self.act.set_succeeded(grasp_result)


if __name__ == '__main__':
    rospy.init_node('object_grasper')
    grasper = ObjectGrasper()
    rospy.spin()
