#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import math
import threading
import time
# ros msgs
from std_msgs.msg import Bool,Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.msg import DynamixelStateList
# ros srvs
from dynamixel_workbench_msgs.srv import DynamixelCommand
from manipulation.srv import ManipulateSrv

class MotorController(object):
    def __init__(self):
        # ROS Topic Subscriber
        rospy.Subscriber('/dynamixel_workbench/dynamixel_state',DynamixelStateList,self.getMotorStateCB)
        # ROS Topic Publisher
        self.motor_pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory',JointTrajectory,queue_size=10)
        # ROS Service Client
        self.motor_client = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command',DynamixelCommand)
        # Motor Parameters
        self.origin_angle = [2048, 2048, 2048, 2048, 2048, 2048]
        self.current_pose = [0]*5
        self.torque_error = [0]*5
        self.rotation_velocity = [0]*5
        
    def getMotorStateCB(self, state):
        for i in range(5):
            self.current_pose[i] = state.dynamixel_state[i].present_position
            self.rotation_velocity[i] = abs(state.dynamixel_state[i].present_velocity)
            self.torque_error[i] = state.dynamixel_state[i].present_current

    def callMotorService(self, motor_id, rotate_value):
        if type(rotate_value) == type(float()):
            rotate_value = self.radToStep(rotate_value)
        res = self.motor_client('', motor_id, 'Goal_Position', rotate_value)

    def radToStep(self,rad):
        return int((rad + math.pi) / (2*math.pi) * 4095)

    def stepToRad(self,step):
        return step / 4095.0 * 2*math.pi - math.pi
    

class JointController(MotorController):
    def __init__(self):
        super(JointController,self).__init__()
        # ROS Topic Subscriber
        rospy.Subscriber('/shoulder_req',Float64,self.shoulderPub)
        rospy.Subscriber('/elbow_req',Float64,self.elbowPub)
        rospy.Subscriber('/wrist_req',Float64,self.wristPub)
        rospy.Subscriber('/endeffector_req',Bool,self.endeffectorPub)
        rospy.Subscriber('/head_req',Float64,self.headPub)

    def shoulderPub(self,rad):
        if type(rad) == type(Float64()):
            rad = rad.data
        step = self.radToStep(rad)
        step0 = 4095 - step
        step1 = step
        print '0:', step0, ' 1:', step1
        thread_m0 = threading.Thread(target=self.callMotorService, args=(0, step0,))
        thread_m1 = threading.Thread(target=self.callMotorService, args=(1, step1,))
        thread_m0.start()
        thread_m1.start()
        rospy.sleep(0.1)
        while (self.rotation_velocity[0] > 0 or self.rotation_velocity[1] > 0) and not rospy.is_shutdown():
            pass
        rospy.sleep(1.0)
        if abs(self.torque_error[0]) > 40 or abs(self.torque_error[1] > 40):
            thread_m0 = threading.Thread(target=self.callMotorService, args=(0, self.current_pose[0]+30,))
            thread_m1 = threading.Thread(target=self.callMotorService, args=(1, self.current_pose[1]-30,))
            thread_m0.start()
            thread_m1.start()

    def elbowPub(self,rad):
        if type(rad) == type(Float64()):
            rad = rad.data
        rad *= -1
        step = self.radToStep(rad)
        print '2: ', step
        self.callMotorService(2, step)
        while self.rotation_velocity[2] > 0 and not rospy.is_shutdown():
            pass
        rospy.sleep(0.1)
        if abs(self.torque_error[2]) > 40:
            self.callMotorService(2, self.current_pose[2]+30)

    def wristPub(self,rad):
        if type(rad) == type(Float64()):
            rad = rad.data
        step = self.radToStep(rad)
        print '3: ', step
        self.callMotorService(3, step)
        while self.rotation_velocity[3] > 0 and not rospy.is_shutdown():
            pass
        rospy.sleep(0.1)
        if abs(self.torque_error[3]) > 40:
            self.callMotorService(3, self.current_pose[3]-30)

    def endeffectorPub(self,req):
        angle = self.origin_angle[4]
        self.callMotorService(4, angle)
        while self.rotation_velocity[4] > 0 and not rospy.is_shutdown():
            pass
        rospy.sleep(0.5)
        grasp_flg = True
        while abs(self.torque_error[4]) <= 100 and not rospy.is_shutdown():
            angle += 30
            self.callMotorService(4, angle)
            rospy.sleep(0.06)
            while self.rotation_velocity[4] > 2.0 and not rospy.is_shutdown():
                pass
            if angle > 2900:
                grasp_flg = False
                break;
        rospy.sleep(0.1)
        self.callMotorService(4, self.current_pose[4]-40)
        print 'fin'
        return grasp_flg

    def headPub(self,rad):
        step = self.radToStep(rad)
        self.callMotorService(5, step)
    
    
class ArmPoseChanger(JointController):
    def __init__(self):
        super(ArmPoseChanger,self).__init__()
        # ROS Topic Subscriber
        rospy.Subscriber('/origin_initialize_req',Bool,self.initializeMotor)
        arm_changer = rospy.Service('/change_arm_pose', ManipulateSrv, self.changeArmPose)

    def initializeMotor(self,res):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = ['m0_controller', 'm1_controller', 'm2_controller', 'm3_controller', 'm4_controller', 'm5_controller']
        msg.points = [JointTrajectoryPoint() for i in range(1)]
        msg.points[0].positions = [self.origin_angle[0], self.origin_angle[1], self.origin_angle[2], self.origin_angle[3], self.origin_angle[4], self.origin_angle[5]]
        msg.points[0].time_from_start = rospy.Time(1.0)
        self.motor_pub.publish(msg)
        rospy.loginfo('initialized all motors!')

    def inverseKinematics(self, x, y):
        l0 = 0.81# Height from ground to shoulder(metre)
        l1 = 0.19# Length from shoulder to elbow(metre)
        l2 = 0.17# Length from elbow to wrist(metre)
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

    def changeArmPose(self, cmd):
        if type(cmd) != str:
            cmd = cmd.target
        rospy.loginfo('Chagnge arm command : %s'%cmd)
        if cmd == 'carry':
            self.carryMode()
            return True
        elif cmd == 'give':
            self.giveMode()
            return True
        elif cmd == 'place':
            self.placeMode()
            return True
        else :
            rospy.loginfo('No sudh change arm command.')
            return False
        
    def carryMode(self):
        shoulder_param = -3.0
        elbow_param = 2.6
        wrist_param =1.4
        self.armController(shoulder_param, elbow_param, wrist_param)

    def giveMode(self):
        shoulder_param = -1.2
        elbow_param = 2.6
        wrist_param = -0.7
        self.armController(shoulder_param, elbow_param, wrist_param)
        while self.rotation_velocity[3] > 0 and not rospy.is_shutdown():
            pass
        rospy.sleep(0.5)
        wrist_error = abs(self.torque_error[3])
        give_time = time.time()
        while wrist_error - abs(self.torque_error[3]) < 0.009 and time.time() - give_time < 5.0 and not rospy.is_shutdown():
            pass
        self.callMotorService(4, self.origin_angle[4])
        rospy.loginfo('Finish give command\n')

    def placeMode(self, height):
        '''
        shoulder_param = 0.0
        elbow_param = 0.0
        wrist_param = 0.0
        self.armController(shoulder_param, elbow_param, wrist_param)
        '''

        
if __name__ == '__main__':
    rospy.init_node('motor_controller')
    experiment = ArmPoseChanger()
    rospy.spin()
