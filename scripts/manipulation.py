#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys
import copy
import rospy
import time
import math
import threading
from std_msgs.msg import Bool,Float64,String
from geometry_msgs.msg import Pose
from dynamixel_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

class Manipulation:
    def __init__(self):
        self.xyz_centroid_sub = rospy.Subscriber('/object/xyz_centroid',Point,self.GraspObjectCB)
        self.changing_pose_req_sub = rospy.Subscriber('/arm/changing_pose_req',String,self.ChangePoseReqCB)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size = 1)
        self.grasp_res_pub = rospy.Publisher('/object/grasp_res',Bool,queue_size = 1)
        self.changing_pose_res_pub = rospy.Publisher('/arm/changing_pose_res',Bool,queue_size = 1)
        self.retry_pub = rospy.Publisher('/object/grasp_req',String,queue_size = 1)
        self.navigation_place_sub = rospy.Subscriber('/navigation/move_place',String,self.NavigationPlace)

        #Definition of Moters    
        self.m0_sub = rospy.Subscriber('/m0_controller/state',JointState,self.M0StateCB)
        self.m1_sub = rospy.Subscriber('/m1_controller/state',JointState,self.M1StateCB)
        self.m2_sub = rospy.Subscriber('/m2_controller/state',JointState,self.M2StateCB)
        self.m3_sub = rospy.Subscriber('/m3_controller/state',JointState,self.M3StateCB)
        self.m4_sub = rospy.Subscriber('/m4_controller/state',JointState,self.M4StateCB)
        self.m0_pub = rospy.Publisher('/m0_controller/command',Float64,queue_size = 1)
        self.m1_pub = rospy.Publisher('/m1_controller/command',Float64,queue_size = 1)
        self.m2_pub = rospy.Publisher('/m2_controller/command',Float64,queue_size = 1)
        self.m3_pub = rospy.Publisher('/m3_controller/command',Float64,queue_size = 1)
        self.m4_pub = rospy.Publisher('/m4_controller/command',Float64,queue_size = 1)
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.LaserCB)

        #Define each moter origin
        self.M0_ORIGIN_ANGLE = -0.562459622225
        self.M1_ORIGIN_ANGLE = 0.628932123033
        self.M2_ORIGIN_ANGLE = 0.317022696163
        self.M3_ORIGIN_ANGLE = 0.0
        self.M4_ORIGIN_ANGLE = 0.5
        
        self.m0_current_pos = 0.00
        self.m0_error = 0.00
        self.m0_is_moving = False
        self.m1_current_pos = 0.00
        self.m1_error = 0.00
        self.m1_is_moving = False
        self.m2_current_pos = 0.00
        self.m2_error = 0.00
        self.m2_is_moving = False
        self.m2_velocity = 0.00
        self.m3_current_pos = 0.00
        self.m3_error = 0.00
        self.m3_is_moving = False
        self.m3_velocity = 0.00
        self.m4_current_pos = 0.00
        self.m4_error = 0.00
        self.m4_is_moving = False
        self.m4_velocity = 0.00
        self.front_laser_dist = 999.9
        self.navigation_place = 'None'

    def ChangePoseReqCB(self,cmd):
        print 'change arm command :', cmd.data
        if cmd.data == 'carry':
            shoulder_param = -3.0
            elbow_param = 2.6
            wrist_param = 1.4
            self.manipulationController(shoulder_param, elbow_param, wrist_param)
        elif cmd.data == 'give':
            shoulder_param = -1.2
            elbow_param = 2.6
            wrist_param = -0.7
            self.manipulationController(shoulder_param, elbow_param, wrist_param)
            while self.m3_is_moving and not rospy.is_shutdown():
                pass
            rospy.sleep(1.0)
            wrist_error = abs(self.m3_error)
            while wrist_error - abs(self.m3_error) < 0.009 and not rospy.is_shutdown():
                pass
            self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
            self.changing_pose_res_pub.publish(True)
            arm_change_cmd = String()
            arm_change_cmd.data = 'carry'
            self.ChangePoseReqCB(arm_change_cmd)
        elif cmd.data == 'place':
#            self.MoveBase(-0.4)
            if self.navigation_place == 'Drawer':
                y = 0.65-0.75
                x = 0.35
                print 'y1 : ', y
            elif self.navigation_place == 'Cupboard':
                y = 0.81-0.75
                x = 0.35
                print 'y2 : ', y
            elif self.navigation_place == 'Couch':
                y = 0.50-0.75
                x = 0.3
                print 'y3 : ', y
            else :
                y = 0.60-0.86
                x = 0.35
            l1 = 0.24# Length from shoulder to elbow(metre)
            l2 = 0.20# Length from elbow to wrist(metre)
            l3 = 0.36# Length of end effector(metre)
            shoulder_angle = -1*math.acos((x*x+y*y+l1*l1-l2*l2) / (2*l1*math.sqrt(x*x+y*y))) + math.atan(y/x)# -1倍の有無で別解
            elbow_angle = math.atan((y-l1*math.sin(shoulder_angle))/(x-l1*math.cos(shoulder_angle)))-shoulder_angle
            wrist_angle = -1*(shoulder_angle + elbow_angle)
            shoulder_angle *= 2.1
            elbow_angle *= 2.1
            self.manipulationController(shoulder_angle, elbow_angle, wrist_angle)
            self.MoveBase(0.6)
            '''
            wrist_param = -0.7
            self. wristPub(wrist_param)
            while self.m3_is_moving and not rospy.is_shutdown():
                pass
            rospy.sleep(0.1)
            m3_angle = self.m3_current_pos
            m3_diff = self.m3_error
            shoulder_param = -1*(1.3+m3_angle)*2.1-(m3_diff+0.01)*32
            # -(m3_diff+0.025)*32：重さ補正
            rospy.sleep(0.2)
            self.shoulderPub(shoulder_param)
            rospy.sleep(0.2)
            elbow_param = 2.6
            self.elbowPub(elbow_param)
            move_range = (self.front_laser_dist-0.75)*3.0
            self.MoveBase(move_range * 0.6)
            rospy.sleep(0.4)
            self.MoveBase(move_range * 0.4)
            rospy.sleep(0.4)
            angle = 0.0
            elbow_current = self.m2_current_pos
            wrist_current = self.m3_current_pos
            wrist_error = self.m3_error + 0.01
            #while self.m3_error <= -0.015 and not rospy.is_shutdown():
            while self.m3_error - wrist_error < 0.015 and not rospy.is_shutdown():
                print (angle*100)%2
                if (angle*100) % 2:
                    wrist_error = self.m3_error
                angle += 0.03
                elbow_angle = elbow_current + angle*2
                wrist_angle = wrist_current + angle
                #self.manipulationController(shoulder_param, elbow_param, wrist_param)
                self.m2_pub.publish(elbow_angle)
                rospy.sleep(0.1)
                self.m3_pub.publish(wrist_angle)
                rospy.sleep(0.05)
                while (self.m2_is_moving or self.m3_is_moving) and not rospy.is_shutdown():
                    pass
                print self.m3_error - wrist_error
                if angle > 1.5:
                    break
                rospy.sleep(0.1)
            '''
            self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
            rospy.sleep(1.0)
            self.MoveBase(-0.3)
            self.shoulderPub(shoulder_param+0.5)
            self.MoveBase(-0.8)
            arm_change_cmd = String()
            arm_change_cmd.data = 'carry'
            self.ChangePoseReqCB(arm_change_cmd)
            arm_change_flg = Bool()
            arm_change_flg.data = True
            self.changing_pose_res_pub.publish(arm_change_flg)
        else :
            print 'No such command.'
                        
    def MoveBase(self,rad_speed):
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

    def GraspObjectCB(self,obj_cog):
        arm_change_cmd = String()
        arm_change_cmd.data = 'carry'
        self.ChangePoseReqCB(arm_change_cmd)
        self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
        obj_cog.y += 0.08 # calibrate RealSenseCamera d435
        #obj_cog.z -= 0.015
        print obj_cog
        if math.isnan(obj_cog.x):
            self.MoveBase(0.6)
            retry_cmd = String()
            retry_cmd.data = 'retry'
            self.retry_pub.publish(retry_cmd)
            return
        obj_angle = math.atan2(obj_cog.y,obj_cog.x)
        print 'obj_angle : ', obj_angle
        if obj_angle < -0.05 or 0.05 <obj_angle:
            print 'There is not object in front.'
            #move kobuki
            #revolve to object
            cmd = Twist()
            cmd.linear.x = 0
            cmd.angular.z = obj_angle * 4.0
            if 0 < cmd.angular.z  and cmd.angular.z < 0.85:
                cmd.angular.z = 0.85
            elif 0 > cmd.angular.z and cmd.angular.z > -0.85:
                cmd.angular.z = -0.85
            print 'cmd.angular.z : ', cmd.angular.z
            self.cmd_vel_pub.publish(cmd)
            time.sleep(3)
            cmd.angular.z = 0
            self.cmd_vel_pub.publish(cmd)
            retry_cmd = String()
            retry_cmd.data = 'retry'
            self.retry_pub.publish(retry_cmd)
            print 'finish roll'
            return
        if obj_cog.x < 0.6 or obj_cog.x > 0.7:
            move_range = (obj_cog.x-0.65)*2.0
            if abs(move_range) < 0.5:
                move_range = int(move_range/abs(move_range))*0.5
            print move_range
            self.MoveBase(move_range)
            retry_cmd = String()
            retry_cmd.data = 'retry'
            self.retry_pub.publish(retry_cmd)
            print 'moving'
            return
        l0 = 0.85# Height from ground to shoulder(metre)
        l1 = 0.24# Length from shoulder to elbow(metre)
        l2 = 0.20# Length from elbow to wrist(metre)
        l3 = 0.36# Length of end effector(metre)
        x = 0.35# obj_cog.x - l3
        #y = obj_cog.z - l0
        if self.navigation_place == 'Drawer':
            y = 0.90-0.65
            x = 0.35
        elif self.navigation_place == 'Cupboard':
            y = 0.77-0.65
            x = 0.37
        elif self.navigation_place == 'Couch':
            y = 0.46-0.65
            x = 0.30
        else :
            y = obj_cog.z - l0
            x = 0.35
        self.MoveBase(-0.6)
        data1 =  x*x+y*y+l1*l1-l2*l2
        data2 =  2*l1*math.sqrt(x*x+y*y)
        print 'y : ', y
        if data1 > data2:
            print 'I can not move arm.'
            retry_cmd = String()
            retry_cmd.data = 'retry'
            self.retry_pub.publish(retry_cmd)
            return
        shoulder_angle = -1*math.acos((x*x+y*y+l1*l1-l2*l2) / (2*l1*math.sqrt(x*x+y*y))) + math.atan(y/x)# -1倍の有無で別解
        elbow_angle = math.atan((y-l1*math.sin(shoulder_angle))/(x-l1*math.cos(shoulder_angle)))-shoulder_angle
        wrist_angle = -1*(shoulder_angle + elbow_angle)
        shoulder_angle *= 2.1
        elbow_angle *= 2.1
        print 'shoulder_angle : ', shoulder_angle, ' deg : ', math.degrees(shoulder_angle)
        print 'elbow_angle    : ', elbow_angle, ' deg : ', math.degrees(elbow_angle)
        print 'wrist_angle    : ', wrist_angle, ' deg : ', math.degrees(wrist_angle)
        self.manipulationController(shoulder_angle*2.1, elbow_angle*2.1, wrist_angle)
        '''
        ### 決め打ち用
        shoulder_angle = -1.07115820647
        elbow_angle = 2.3719040394
        wrist_angle = -0.619402777586
        self.manipulationController(shoulder_angle, elbow_angle, wrist_angle)
        ###
        '''
        rospy.sleep(3.0)
        move_range = (0.17+obj_cog.x+0.15-(x+0.2))*3.1
        self.MoveBase(move_range*0.7)
        rospy.sleep(0.4)
        self.MoveBase(move_range*0.3)
        grasp_flg = self.endeffectorPub(True)
        rospy.sleep(2.0)
        self.shoulderPub(shoulder_angle+0.5)
        self.MoveBase(-0.6)
        self.shoulderPub(0.7)
        arm_change_cmd.data = 'carry'
        self.ChangePoseReqCB(arm_change_cmd)
        rospy.sleep(4.0)
        if grasp_flg :
            grasp_flg = self.m4_error > 0.03
            print 'grasp flg : ', grasp_flg
        if grasp_flg :
            self.grasp_res_pub.publish(grasp_flg)
        else:
            retry_cmd = String()
            retry_cmd.data = 'retry'
            self.retry_pub.publish(retry_cmd)
            return
        self.MoveBase(-0.4)
        print 'finish'

    def manipulationController(self, shoulder_param, elbow_param, wrist_param):
        thread_shoulder = threading.Thread(target=self.shoulderPub, args=(shoulder_param,))
        thread_elbow = threading.Thread(target=self.elbowPub, args=(elbow_param,))
        thread_wrist = threading.Thread(target=self.wristPub, args=(wrist_param,))
        thread_elbow.start()
        rospy.sleep(0.2)
        thread_wrist.start()
        rospy.sleep(0.2)
        thread_shoulder.start()
        
    def shoulderPub(self, m01):
        m0_p = self.M0_ORIGIN_ANGLE - m01
        m1_p = self.M1_ORIGIN_ANGLE + m01
        self.m0_pub.publish(m0_p)
        self.m1_pub.publish(m1_p)
        rospy.sleep(0.1)
        while self.m0_is_moving or self.m1_is_moving:
            pass
        rospy.sleep(0.1)
        if self.m1_error > 0.06 or self.m0_error > 0.06:
            self.m1_pub.publish(self.m1_current_pos)
            self.m0_pub.publish(self.m0_current_pos)

    def elbowPub(self, m2):
        m2 *= -1
        m2_p = self.M2_ORIGIN_ANGLE + m2
        self.m2_pub.publish(m2_p)
        rospy.sleep(0.1)
        while self.m2_is_moving:
            pass
        if abs(self.m2_error) > 0.06:
            self.m2_pub.publish(self.m2_current_pos)

    def wristPub(self, m3):
        m3_p = self.M3_ORIGIN_ANGLE + m3
        self.m3_pub.publish(m3_p)
        rospy.sleep(0.1)
        while self.m3_is_moving:
            pass
        if abs(self.m3_error) > 0.06:
            self.m3_pub.publish(self.m3_current_pos)

    def endeffectorPub(self,req):
        angle = self.M4_ORIGIN_ANGLE
        self.m4_pub.publish(angle)
        rospy.sleep(0.1)
        while self.m4_is_moving and not rospy.is_shutdown():
            pass
        rospy.sleep(0.1)
        grasp_flg = True
        while self.m4_error <= 0.09 and not rospy.is_shutdown():
            angle -= 0.05
            self.m4_pub.publish(angle)
            rospy.sleep(0.09)
            while self.m4_velocity >= 2.0:
                pass
            if angle < -0.3:
                grasp_flg = False
                break
        rospy.sleep(0.1)
        self.m4_pub.publish(self.m4_current_pos -  0.06)
        return grasp_flg
            
    def M0StateCB(self,state):
        self.m0_current_pos = state.current_pos
        self.m0_error = abs(state.error)
        self.m0_is_moving = state.is_moving

    def M1StateCB(self,state):
        self.m1_current_pos = state.current_pos
        self.m1_error = abs(state.error)
        self.m1_is_moving = state.is_moving

    def M2StateCB(self,state):
        self.m2_current_pos = state.current_pos
        self.m2_error = state.error
        self.m2_is_moving = state.is_moving
        self.m2_velocity = abs(state.velocity)

    def M3StateCB(self,state):
        self.m3_current_pos = state.current_pos
        self.m3_error = state.error
        self.m3_is_moving = state.is_moving
        self.m3_velocity = abs(state.velocity)
        
    def M4StateCB(self,state):
        self.m4_current_pos = state.current_pos
        self.m4_error = abs(state.error)
        self.m4_is_moving = state.is_moving
        self.m4_velocity = abs(state.velocity)
    
    def LaserCB(self,laser_scan):
        self.front_laser_dist = laser_scan.ranges[360]
        #print self.front_laser_dist

    def NavigationPlace(self,res):
        self.navigation_place = res.data

if __name__ == '__main__':
    rospy.init_node('Manipulation',anonymous=True)
    manipulation = Manipulation()
    rospy.spin()
