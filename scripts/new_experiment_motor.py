#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import math
import threading
# ros msgs
from std_msgs.msg import Bool,Float64,String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msg.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand

class Experiment(object):
    def __init__(self):
        # topic subscriber
        # topic publisher
        self.m0_pub = rospy.Publisher('m0_controller/command',Float64,queue_size=1)
        self.m1_pub = rospy.Publisher('m1_controller/command',Float64,queue_size=1)
        self.m2_pub = rospy.Publisher('m2_controller/command',Float64,queue_size=1)
        self.m3_pub = rospy.Publisher('m3_controller/command',Float64,queue_size=1)
        self.m4_pub = rospy.Publisher('m4_controller/command',Float64,queue_size=1)
        self.m6_pub = rospy.Publisher('m6_controller/command',Float64,queue_size=1)




class MotorController(object):
    def __init__(self):
        # ROS Topic Subscriber
        rospy.Subscriber('/dynamixel_workbench/joint_states',JointState,self.getMotorStatesCB)
        # ROS Topic Publisher
        self.motor_pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory',JointTrajectory,queue_size=10)
        # ROS Service Client
        self.motor_client = rospy.Service('/dynamixel_workbench/dynamixel_command',DynamixelCommand)## Args:command id addr_name value
        # Motor Parameters
        self.origin_angle = []
        self.current_pos = [0]*5
        self.torque_error = [0]*5
        self.rotation_velocity = [0]*5
        
        self.M0_ORIGIN_ANGLE = -0.562459622225
        self.M1_ORIGIN_ANGLE = 0.628932123033
        self.M2_ORIGIN_ANGLE = 0.317022696163
        self.M3_ORIGIN_ANGLE = 0.0
        self.M4_ORIGIN_ANGLE = 0.5
        self.M6_ORIGIN_ANGLE = 0.3
        
    def getMotorStateCB(self, state):
        for i in range(5):
            self.current_pos[i] = state.position[i]
            self.torque_error[i] = state.effort[i]
            self.rotation_velocity[i] = state.velocity[i]

    def callMotorService(self, motor_id, value):
        msg = DynamixelCommand()
        msg.id = 
        res = self.motor_client()
    

class ArmController():
    def __init__(self):
        rospy.Subscriber('/origin_initialize_req',Bool,self.initializeMotor)
        rospy.Subscriber('/head_req',Float64,self.headPub)
        rospy.Subscriber('/shoulder_req',Float64,self.shoulderPub)
        rospy.Subscriber('/elbow_req',Float64,self.elbowPub)
        rospy.Subscriber('/wrist_req',Float64,self.wristPub)
        rospy.Subscriber('/endeffector_req',Bool,self.endeffectorPub)

    def inverseKinematics(self, x, y):
        l0 = 0.81# Height from ground to shoulder(metre)
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
            #self.armController(shoulder_angle, elbow_angle, wrist_angle)
            return [shoulder_angle, elbow_angle, wrist_angle]
        except ValueError:
            rospy.loginfo('I can not move arm.')
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
        
    def initializeMotor(self,res):
        self.m0_pub.publish(self.M0_ORIGIN_ANGLE)
        self.m1_pub.publish(self.M1_ORIGIN_ANGLE)
        self.m2_pub.publish(self.M2_ORIGIN_ANGLE)
        self.m3_pub.publish(self.M3_ORIGIN_ANGLE)
        self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
        rospy.loginfo('initialized all motors!')

    def shoulderPub(self,m01):
        if type(m01) == type(Float64()):
            m01 = m01.data
        m0_p = self.M0_ORIGIN_ANGLE - m01
        m1_p = self.M1_ORIGIN_ANGLE + m01
        self.m0_pub.publish(m0_p)
        self.m1_pub.publish(m1_p)
        rospy.sleep(0.1)
        while self.m0_is_moving or self.m1_is_moving:
            pass
        rospy.sleep(0.1)
        if self.m1_error > 0.06 or self.m0_error > 0.06:
            self.m1_pub.publish(self.m1_current_pos+0.04)
            self.m0_pub.publish(self.m0_current_pos-0.04)

    def elbowPub(self,m2):
        if type(m2) == type(Float64()):
            m2 = m2.data
        m2 *= -1
        m2_p = self.M2_ORIGIN_ANGLE + m2
        self.m2_pub.publish(m2_p)
        rospy.sleep(0.1)
        while self.m2_is_moving:
            pass
        rospy.sleep(0.1)
        if abs(self.m2_error) > 0.06:
            self.m2_pub.publish(self.m2_current_pos+0.04)

    def wristPub(self,m3):
        if type(m3) == type(Float64()):
            m3 = m3.data
        m3_p = self.M3_ORIGIN_ANGLE + m3
        self.m3_pub.publish(m3_p)
        rospy.sleep(0.1)
        while self.m3_is_moving:
            pass
        rospy.sleep(0.1)
        if abs(self.m3_error) > 0.06:
            self.m3_pub.publish(self.m3_current_pos-0.04)

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
            if angle < -0.6:
                grasp_flg = False
                break;
        rospy.sleep(0.1)
        self.m4_pub.publish(self.m4_current_pos -  0.05)
        return grasp_flg

    
class ConnectFrame():
    def __init__(self):
        pass

    
if __name__ == '__main__':
    rospy.init_node('motor_controller')
    experiment = Experiment()
    rospy.spin()
