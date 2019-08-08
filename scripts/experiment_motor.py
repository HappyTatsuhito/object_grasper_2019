#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import threading
from std_msgs.msg import Bool,Float64,String
from dynamixel_msgs.msg import JointState

class Manipulation:
    def __init__(self):
        self.origin_initializer_sub = rospy.Subscriber('/origin_initialize_req',Bool,self.initializeMotorCB)
        self.changing_pose_req_sub = rospy.Subscriber('/arm/changing_pose_req',String,self.ChangePoseReqCB)
        self.shoulder_sub = rospy.Subscriber('/shoulder_req',Float64,self.shoulderCB)
        self.elbow_sub = rospy.Subscriber('/elbow_req',Float64,self.elbowCB)
        self.wrist_sub = rospy.Subscriber('/wrist_req',Float64,self.wristCB)
        self.endeffector_sub = rospy.Subscriber('/endeffector_req',Bool,self.endeffectorCB)
        self.m0_sub = rospy.Subscriber('/m0_controller/state',JointState,self.M0StateCB)
        self.m1_sub = rospy.Subscriber('/m1_controller/state',JointState,self.M1StateCB)
        self.m2_sub = rospy.Subscriber('/m2_controller/state',JointState,self.M2StateCB)
        self.m3_sub = rospy.Subscriber('/m3_controller/state',JointState,self.M3StateCB)
        self.m4_sub = rospy.Subscriber('/m4_controller/state',JointState,self.M4StateCB)
        self.m0_pub = rospy.Publisher('m0_controller/command',Float64,queue_size = 1)
        self.m1_pub = rospy.Publisher('m1_controller/command',Float64,queue_size = 1)
        self.m2_pub = rospy.Publisher('m2_controller/command',Float64,queue_size = 1)
        self.m3_pub = rospy.Publisher('m3_controller/command',Float64,queue_size = 1)
        self.m4_pub = rospy.Publisher('m4_controller/command',Float64,queue_size = 1)

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

    def initializeMotorCB(self,res):
        self.m0_pub.publish(self.M0_ORIGIN_ANGLE)
        self.m1_pub.publish(self.M1_ORIGIN_ANGLE)
        self.m2_pub.publish(self.M2_ORIGIN_ANGLE)
        self.m3_pub.publish(self.M3_ORIGIN_ANGLE)
        self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
        print 'initialized all motors!'

    def ChangePoseReqCB(self,cmd):
        print "command is", cmd.data
        shoulder_param = Float64()
        elbow_param = Float64()
        wrist_param = Float64()
        endeffector_param = Float64()
        if cmd.data == 'carry':
            shoulder_param.data = -3.0
            elbow_param.data = 2.6
            wrist_param.data = 1.4
            self.manipulationController(shoulder_param, elbow_param, wrist_param)
        elif cmd.data == 'pass':
            shoulder_param.data = -1.2
            elbow_param.data = 2.6
            wrist_param.data = -0.7
            self.manipulationController(shoulder_param, elbow_param, wrist_param)
            while self.m3_is_moving and not rospy.is_shutdown():
                pass
            rospy.sleep(1.0)
            wrist_error = abs(self.m3_error)
            while wrist_error - abs(self.m3_error) < 0.009 and not rospy.is_shutdown():
                pass
            self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
        elif cmd.data == 'place':
            '''
            self.MoveBase(-0.6)
            shoulder_param.data = -1.2
            elbow_param.data = 2.6
            wrist_param.data = -0.7
            self.manipulationController(shoulder_param, elbow_param, wrist_param)
            rospy.sleep(2.0)
            move_range = (self.front_laser_dist-0.75)*2.5
            self.MoveBase(move_range * 0.6)
            rospy.sleep(0.4)
            self.MoveBase(move_range * 0.4)
            self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
            rospy.sleep(1.5)
            self.MoveBase(-0.5)
            arm_change_cmd = String()
            arm_change_cmd.data = 'carry'
            self.ChangePoseReqCB(arm_change_cmd)
            arm_change_flg = Bool()
            arm_change_flg.data = True
            self.changing_pose_res_pub.publish(arm_change_flg)
            '''
            ###
            wrist_param.data = -0.7
            self.wristCB(wrist_param)
            while self.m3_is_moving and not rospy.is_shutdown():
                pass
            rospy.sleep(0.1)
            m3_angle = self.m3_current_pos
            m3_diff = self.m3_error
            print m3_angle
            print m3_diff
            shoulder_param.data = -1*(1.3+m3_angle)*2.1-(m3_diff+0.01)*32
            # -(m3_diff+0.025)*32：重さ補正
            rospy.sleep(0.1)
            self.shoulderCB(shoulder_param)
            rospy.sleep(0.1)
            elbow_param.data = 2.6
            self.elbowCB(elbow_param)
            '''
            rospy.sleep(2.0)
            angle = 0.0
            print '1'
            elbow_c = self.m2_current_pos
            wrist_c = self.m3_current_pos
            while self.m3_error <= -0.015 and not rospy.is_shutdown():
                angle += 0.01
                elbow_angle = elbow_c + angle*2
                wrist_angle = wrist_c + angle
                #self.manipulationController(shoulder_param, elbow_param, wrist_param)
                self.m2_pub.publish(elbow_angle)
                rospy.sleep(0.1)
                self.m3_pub.publish(wrist_angle)
                rospy.sleep(0.05)
                while self.m2_velocity >= 1.0 or self.m3_velocity >= 1.0:
                    pass
                if angle > 1.5:
                    break
            print 'm2_e : ', self.m2_error, ' m3_e : ', self.m3_error
            print 'angle : ', angle
            print 'out'
            self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
            '''
            ###

    def manipulationController(self, shoulder_param, elbow_param, wrist_param):
        thread_shoulder = threading.Thread(target=self.shoulderCB, args=(shoulder_param,))
        thread_elbow = threading.Thread(target=self.elbowCB, args=(elbow_param,))
        thread_wrist = threading.Thread(target=self.wristCB, args=(wrist_param,))
        thread_elbow.start()
        rospy.sleep(0.1)
        thread_wrist.start()
        rospy.sleep(0.1)
        thread_shoulder.start()
            
    def shoulderCB(self,m01):
        print 'shoulder parameter : ',  m01.data
        m0_p = self.M0_ORIGIN_ANGLE - m01.data
        m1_p = self.M1_ORIGIN_ANGLE + m01.data
        self.m0_pub.publish(m0_p)
        self.m1_pub.publish(m1_p)
        rospy.sleep(0.1)
        while self.m0_is_moving or self.m1_is_moving:
            pass
        rospy.sleep(0.1)
        if self.m1_error > 0.06 or self.m0_error > 0.06:
            self.m1_pub.publish(self.m1_current_pos+0.04)
            self.m0_pub.publish(self.m0_current_pos-0.04)

    def elbowCB(self,m2):
        m2.data *= -1
        print 'elbow parameter : ', m2.data
        m2_p = self.M2_ORIGIN_ANGLE + m2.data
        self.m2_pub.publish(m2_p)
        rospy.sleep(0.1)
        while self.m2_is_moving:
            pass
        rospy.sleep(0.1)
        if abs(self.m2_error) > 0.06:
            self.m2_pub.publish(self.m2_current_pos+0.04)

    def wristCB(self,m3):
        print 'wrist parameter : ', m3.data
        m3_p = self.M3_ORIGIN_ANGLE + m3.data
        self.m3_pub.publish(m3_p)
        rospy.sleep(0.1)
        while self.m3_is_moving:
            pass
        rospy.sleep(0.1)
        if abs(self.m3_error) > 0.06:
            self.m3_pub.publish(self.m3_current_pos-0.04)

    def endeffectorCB(self,m4):
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
            rospy.sleep(0.1)
            while self.m4_velocity >= 2.0:
                pass
            if angle < -0.3:
                grasp_flg = False
                break;
        rospy.sleep(0.1)
        print 'm4_error : ',self.m4_error
        print 'm4_angle : ', angle
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

if __name__ == '__main__':
    rospy.init_node('ManipulationTest',anonymous=True)
    manipulation = Manipulation()
    rospy.spin()
