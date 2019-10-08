#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import time
import math
import threading
from std_msgs.msg import Bool,Float64,String
from dynamixel_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from object_grasper import ObjectGrasper

class GeneralGrasper(ObjectGrasper):
    def __init__(self):
        super(GeneralGrasper,self).__init__()
        self.navigation_place_sub = rospy.Subscriber('/navigation/move_place',String,self.navigationPlaceCB)

        # instance variables
        self.navigation_place = 'Null'
        self.target_place = {'Null':0.75, 'Eins':0.72, 'Zwei':0.67, 'Drei':0.69}

    def changePoseReqCB(self,cmd):
        print 'Change arm command :', cmd.data
        if cmd.data == 'carry':
            shoulder_param = -3.0
            elbow_param = 2.6
            wrist_param = 1.4
            self.armController(shoulder_param, elbow_param, wrist_param)
        elif cmd.data == 'give':
            shoulder_param = -1.2
            elbow_param = 2.6
            wrist_param = -0.7
            self.armController(shoulder_param, elbow_param, wrist_param)
            while self.m3_is_moving and not rospy.is_shutdown():
                pass
            rospy.sleep(1.0)
            wrist_error = abs(self.m3_error)
            give_time = time.time()
            while wrist_error - abs(self.m3_error) < 0.009 and time.time() - give_time < 5.0 and not rospy.is_shutdown():
                pass
            self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
            self.changing_pose_res_pub.publish(True)
            arm_change_cmd = String()
            arm_change_cmd.data = 'carry'
            self.changePoseReqCB(arm_change_cmd)
            print 'Finish giving\n'
        elif cmd.data == 'place':
            self.moveBase(-0.4)
            y = self.target_place[self.navigation_place] + 0.1
            x = (y-0.75)/10+0.5
            print 'x : ', x
            print 'y : ', y
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
            arm_change_cmd = String()
            arm_change_cmd.data = 'carry'
            self.changePoseReqCB(arm_change_cmd)
            self.navigation_place = 'Null'
            self.changing_pose_res_pub.publish(True)
            print 'Finish placing\n'
        else :
            print 'No such command.'

    def graspObject(self, obj_cog):
        print '-- Grasp Object --'
        print self.navigation_place
        self.moveBase(-0.6)
        if self.navigation_place == 'Null':
            y = obj_cog.z+0.06
        else:
            y = self.target_place[self.navigation_place]+0.1
        x = (y-0.75)/10+0.5 #0.5
        joint_angle = self.inverseKinematics(x, y)
        if not joint_angle:
            return
        self.armController(joint_angle[0], joint_angle[1], joint_angle[2])
        rospy.sleep(3.0)
        move_range = (0.17+obj_cog.x+0.15-(x+0.2))*3.1
        self.moveBase(move_range*0.7)
        rospy.sleep(0.3)
        self.moveBase(move_range*0.3)
        grasp_flg = self.endeffectorPub(True)
        if not grasp_flg:
            self.retry_pub.publish('retry')
            print 'Failed to grasp the object.'
            return
        rospy.sleep(2.0)
        self.shoulderPub(joint_angle[0]+0.5)
        self.moveBase(-0.6)
        self.shoulderPub(0.7)
        arm_change_cmd = String()
        arm_change_cmd.data = 'carry'
        self.changePoseReqCB(arm_change_cmd)
        rospy.sleep(4.0)
        if grasp_flg :
            grasp_flg = self.m4_error > 0.03
        if grasp_flg :
            self.grasp_res_pub.publish(grasp_flg)
            print 'Successfully grasped the object!'
        elif self.grasp_count == 3:
            self.grasp_res_pub.publish(False)
            print 'Failed to grasp the object.'
        else:
            self.grasp_count += 1
            self.retry_pub.publish('retry')
            print 'Failed to grasp the object.'
            return
        self.moveBase(-0.4)
        self.grasp_count = 0
        self.navigation_place = 'Null'
        return
        
    def navigationPlaceCB(self,res):
        target_dic = {'Eins':['desk'], 'Zwei':['cupboard'], 'Drei':['shelf']}
        for key,value in target_dic.items():
            if res.data in value:
                self.navigation_place = key
                break
            else:
                self.navigation_place = 'Null'

    def main(self,obj_cog):
        realsense_hight = 1.00
        m6_angle = -1*(math.atan2(realsense_hight-self.target_place[self.navigation_place], 0.65)-self.M6_ORIGIN_ANGLE)
        self.m6_pub.publish(m6_angle)
        localize_flg = self.localizeObject(obj_cog)
        if localize_flg:
            self.graspObject(obj_cog)
        print 'Finish grasp\n'
                
if __name__ == '__main__':
    rospy.init_node('General_Grasper',anonymous=True)
    grasper = GeneralGrasper()
    rospy.spin()