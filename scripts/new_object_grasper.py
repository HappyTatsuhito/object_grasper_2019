#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import time
import math
import threading
# ros msgs
from std_msgs.msg import Bool,Float64,String
from dynamixel_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
#from sensor_msgs.msg import LaserScan
# action msgs
from manipulation.msg import *
# class inheritance
from experiment_motor import Experiment

class ObjectGrasper(Experiment):
    def __init__(self):
        super(ObjectGrasper,self).__init__()
        # topic subscriber
        changing_pose_req_sub = rospy.Subscriber('/arm/changing_pose_req',String,self.changePoseReqCB)
        navigation_place_sub = rospy.Subscriber('/navigation/move_place',String,self.navigationPlaceCB)
        #self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.laserCB)
        # topic publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size = 1)
        self.changing_pose_res_pub = rospy.Publisher('/arm/changing_pose_res',Bool,queue_size = 1)
        # action server
        self.act = actionlib.SimpleActionServer('/object/grasp',
                                                ObjectGrasperAction,
                                                execute_cb = self.actionMain,
                                                auto_start = False)
        self.act.register_preempt_callback(self.actionPreempt)
        # instance variables
        #self.front_laser_dist = 0.00
        self.navigation_place = 'Null'
        self.target_place = {'Null':0.75, 'Eins':0.72, 'Zwei':0.67, 'Drei':0.69}

        self.act.start()

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

    def inverseKinematics(self, x, y):
        l0 = 0.75# Height from ground to shoulder(metre)
        l1 = 0.24# Length from shoulder to elbow(metre)
        l2 = 0.20# Length from elbow to wrist(metre)
        l3 = 0.15# Length of end effector(metre)
        x -= l3
        y -= l0
        data1 =  x*x+y*y+l1*l1-l2*l2
        data2 =  2*l1*math.sqrt(x*x+y*y)
        try:
            shoulder_angle = -1*math.acos((x*x+y*y+l1*l1-l2*l2) / (2*l1*math.sqrt(x*x+y*y))) + math.atan(y/x)# -1倍の有無で別解
            elbow_angle = math.atan((y-l1*math.sin(shoulder_angle))/(x-l1*math.cos(shoulder_angle)))-shoulder_angle
            wrist_angle = -1*(shoulder_angle + elbow_angle)
            shoulder_angle *= 2.1
            elbow_angle *= 2.1
            print 'shoulder_angle : ', shoulder_angle, ' deg : ', math.degrees(shoulder_angle)
            print 'elbow_angle    : ', elbow_angle, ' deg : ', math.degrees(elbow_angle)
            print 'wrist_angle    : ', wrist_angle, ' deg : ', math.degrees(wrist_angle)
            #self.armController(shoulder_angle, elbow_angle, wrist_angle)
            return [shoulder_angle, elbow_angle, wrist_angle]
        except ValueError:
            print 'I can not move arm.'
            self.retry_pub.publish('Retry')
            return False
        
    def armController(self, shoulder_param, elbow_param, wrist_param):
        thread_shoulder = threading.Thread(target=self.shoulderPub, args=(shoulder_param,))
        thread_elbow = threading.Thread(target=self.elbowPub, args=(elbow_param,))
        thread_wrist = threading.Thread(target=self.wristPub, args=(wrist_param,))
        thread_elbow.start()
        rospy.sleep(0.2)
        thread_wrist.start()
        rospy.sleep(0.2)
        thread_shoulder.start()

    '''
    def laserCB(self,laser_scan):
        self.front_laser_dist = laser_scan.ranges[360]
        #print self.front_laser_dist
    '''
    
    def apploachObject(self,object_centroid):
        if object_centroid.x < 0.6 or object_centroid.x > 0.7:
            move_range = (object_centroid.x-0.65)*2.0
            if abs(move_range) < 0.5:
                move_range = int(move_range/abs(move_range))*0.5
            self.moveBase(move_range)
            return False
        else :
            return True

    def graspObject(self, object_centroid):
        print '-- Grasp Object --'
        self.moveBase(-0.6)
        if self.navigation_place == 'Null':
            y = object_centroid.z + 0.06
        else:
            y = self.target_place[self.navigation_place] + 0.1
        self.navigation_place = 'Null'
        x = (y-0.75)/10+0.5 #0.5
        joint_angle = self.inverseKinematics(x, y)
        if not joint_angle:
            return False
        self.armController(joint_angle[0], joint_angle[1], joint_angle[2])
        rospy.sleep(2.5)
        move_range = (0.17+object_centroid.x+0.15-(x+0.2))*3.1
        self.moveBase(move_range*0.7)
        rospy.sleep(0.3)
        self.moveBase(move_range*0.3)
        grasp_flg = self.endeffectorPub(True)
        rospy.sleep(1.0)
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
            print 'Successfully grasped the object!'
        else:
            print 'Failed to grasp the object.'
        print 'Finish grasp'
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
        rospy.loginfo('preempt callback')
        self.act.set_preempted(text = 'message for preempt')
        self.preempt_flg = True

    def actionMain(self,object_centroid):
        target_centroid = object_centroid.grasp_goal
        #grasp_feedback = ObjectGrasperFeedback()
        grasp_result = ObjectRecognizerResult()
        grasp_flg = False
        approach_flg = self.approachObject(target_centroid)
        if approach_flg:
            grasp_flg = self.graspObject(object_centroid)
        self.grasp_res_pub.publish(grasp_flg)
        grasp_result.grasp_result = grasp_flg
        self.act.set_succeeded(grasp_flg)

        return

if __name__ == '__main__':
    rospy.init_node('Object_Grasper')
    grasper = ObjectGrasper()
    rospy.spin()
