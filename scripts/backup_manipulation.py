#!/usr/bin/env python

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
        self.changing_pose_req_sub = rospy.Subscriber('/arm/changing_pose_req',String,self.ChangePoseReqCB)
        self.xyz_centroid_sub = rospy.Subscriber('/object/xyz_centroid',Point,self.GraspObjectCB)
        #self.place_req_sub = rospy.Subscriber('/arm/place_req',Bool,self.PlaceReqCB)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size = 1)
        #self.retry_pub = rospy.Publisher('/object/recog_req',String,queue_size = 1)
        self.grasp_res_pub = rospy.Publisher('/object/grasp_res',Bool,queue_size = 1)
        self.changing_pose_res_pub = rospy.Publisher('/arm/changing_pose_res',Bool,queue_size = 1)
        self.retry_pub = rospy.Publisher('/object/grasp_req',String,queue_size = 1)
        #self.place_pub = rospy.Publisher('/object/place_res',Bool,queue_size = 1)

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
        #self.m5_pub = rospy.Publisher('/m5_controller/command',Float64,queue_size = 1)
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.LaserCB)

        #Define each moter origin
        self.M0_ORIGIN_ANGLE = -0.562459622225
        self.M1_ORIGIN_ANGLE = 0.628932123033
        self.M2_ORIGIN_ANGLE = 0.317022696163
        self.M3_ORIGIN_ANGLE = 0.0
        self.M4_ORIGIN_ANGLE = 0.5
        
        self.m0_angle = 0.00
        self.m1_angle = 0.00
        self.m0_current_pos = 0.00
        self.m0_error = 0.00
        self.m0_is_moving = False
        self.m1_current_pos = 0.00
        self.m1_error = 0.00
        self.m1_is_moving = False
        self.m2_current_pos = 0.00
        self.m2_error = 0.00
        self.m2_is_moving = False
        self.m3_current_pos = 0.00
        self.m3_error = 0.00
        self.m3_is_moving = False
        self.m4_current_pos = 0.00
        self.m4_error = 0.00
        self.m4_is_moving = False
        self.m4_velocity = 0.00
        self.front_laser_dist = 999.9

    def ChangePoseReqCB(self,cmd):
        print "command is",cmd
        if cmd.data == 'carry':
            #thread_shoulder = threading.Thread(target=self.shoulderPub, args=(-2.1,))
            thread_shoulder = threading.Thread(target=self.shoulderPub, args=(-3.0,))
            thread_elbow = threading.Thread(target=self.elbowPub, args=(2.6,))
            thread_wrist = threading.Thread(target=self.wristPub, args=(1.4,))
            thread_elbow.start()
            rospy.sleep(0.1)
            thread_wrist.start()
            rospy.sleep(0.6)
            thread_shoulder.start()
        elif cmd.data == 'grasp':
            thread_shoulder = threading.Thread(target=self.shoulderPub, args=(-3.0,))
            thread_elbow = threading.Thread(target=self.elbowPub, args=(2.6,))
            thread_wrist = threading.Thread(target=self.wristPub, args=(1.4,))
            thread_wrist.start()
            rospy.sleep(0.1)
            thread_elbow.start()
            rospy.sleep(0.1)
            thread_shoulder.start()
        elif cmd.data == 'place':
            self.MoveBase(-0.6)
            thread_shoulder = threading.Thread(target=self.shoulderPub, args=(-0.7,))
            thread_elbow = threading.Thread(target=self.elbowPub, args=(2.6,))
            thread_wrist = threading.Thread(target=self.wristPub, args=(-0.5,))
            thread_wrist.start()
            rospy.sleep(0.1)
            thread_elbow.start()
            rospy.sleep(0.1)
            thread_shoulder.start()
            rospy.sleep(2.0)
            move_range = (self.front_laser_dist-0.75)*2.6
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

    def PlaceReqCB(self,req):
        self.m0_pub.publish(self.M0_ORIGIN_ANGLE-0.480647313538)
        self.m1_pub.publish(self.M1_ORIGIN_ANGLE+0.424401351315)
        self.m2_pub.publish(self.M2_ORIGIN_ANGLE-1.55954713435)
        self.m3_pub.publish(self.M3_ORIGIN_ANGLE-1.02776712788)
        #self.m4_pub.publish(self.M4_ORIGIN_ANGLE+0.0)
        time.sleep(3)
        self.MoveBase(0.7)
        time.sleep(3)
        self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
        self.place_pub.Publish(True)

    def GraspObjectCB(self,obj_cog):
        arm_change_cmd = String()
        arm_change_cmd.data = 'grasp'
        self.ChangePoseReqCB(arm_change_cmd)
        obj_cog.y += 0.08
        obj_cog.z -= 0.1
        print obj_cog
        #add for qualification video
        #obj_cog.z = obj_cog.z+0.05
        #---
        if math.isnan(obj_cog.x):
            self.MoveBase(0.3)
            retry_cmd = String()
            retry_cmd.data = 'retry'
            self.retry_pub.publish(retry_cmd)
            return
        obj_angle = math.atan2(obj_cog.y,obj_cog.x)
        print 'obj_angle : ', obj_angle
        if obj_angle < -0.05 or 0.05 <obj_angle:
            print "There is not object in front."
            #move kobuki
            #revolve to object
            cmd = Twist()
            cmd.linear.x = 0
            cmd.angular.z = obj_angle*3.5
            if 0 < cmd.angular.z  and cmd.angular.z < 0.6:
                cmd.angular.z = 0.6
            elif 0 > cmd.angular.z and cmd.angular.z > -0.6:
                cmd.angular.z = -0.6
            print 'cmd.angular.z is'
            print cmd.angular.z
            self.cmd_vel_pub.publish(cmd)
            time.sleep(3)
            cmd.angular.z = 0
            self.cmd_vel_pub.publish(cmd)
            retry_cmd = String()
            retry_cmd.data = 'retry'
            self.retry_pub.publish(retry_cmd)
            print "finish roll"
            return
        l0 = 0.85#Height from ground to shoulder(metre)
        l1 = 0.24#Length from shoulder to elbow(metre)
        l2 = 0.20#Length from elbow to wrist(metre)
        l3 = 0.36#Length of end effector(metre)
        x = obj_cog.x - l3
        y = obj_cog.z - l0
        dist = x*x+y*y
        if dist < l1*l1+l2*l2:
            self.MoveBase((dist - (l1*l1+l2*l2))*2-0.3)
            gpsr_cmd = String()
            gpsr_cmd.data = 'retry'
            self.retry_pub.publish(gpsr_cmd)
            print "finish back"
        elif (l1+l2)*(l1+l2) < dist:
            print dist - (l1+l2)*(l1+l2)
            self.MoveBase((dist - (l1+l2)*(l1+l2))*2+0.3)
            retry_cmd = String()
            retry_cmd.data = 'retry'
            self.retry_pub.publish(retry_cmd)
            print "finish advance"
        else:
            #calculate inverse kinematicst!
            joint0_angle = 0
            joint1_angle = 0
            joint2_angle = 0
            singlarity_flg = False
            
            try:
                x0 = 0.37
                '''
                joint0_angle = -1*math.acos((x0*x0+y*y+l1*l1+l2*l2) / (2*l1*math.sqrt(x0*x0+y*y))) + math.atan(y/x0)
                joint1_angle = math.acos(((x0*x0+y*y)-(l1*l1+l2*l2)) / (2*l1*l2))
                joint2_angle = -1*(joint1_angle + joint0_angle)
                print 'joint0_angle : ', joint0_angle
                print 'joint1_angle : ', joint1_angle
                print 'joint2_angle : ', joint2_angle
                '''
            except:
                print 'the end points is singlarity'
                singlarity_flg = True
            if joint0_angle < -1.57 or 1.57 < joint0_angle:
                singlarity_flg = True
            if joint1_angle < -1.57 or 1.57 < joint1_angle:
                singlarity_flg = True
            if joint2_angle < -1.57 or 1.57 < joint2_angle:
                singlarity_flg = True
            if singlarity_flg == True:
                print 'I can not move arm'
                return
            self.MoveBase(-0.5)#0.7
            data1 =  x0*x0+y*y+l1*l1-l2*l2
            data2 =  2*l1*math.sqrt(x0*x0+y*y)
            print 'y', y
            if data1 < data2:
                joint0_angle1 = math.acos((x0*x0+y*y+l1*l1-l2*l2) / (2*l1*math.sqrt(x0*x0+y*y))) + math.atan(y/x0)
                joint1_angle1 = math.atan((y-l1*math.sin(joint0_angle1))/(x-l1*math.cos(joint0_angle1)))-joint0_angle1
                joint2_angle1 = -1*(joint0_angle1 + joint1_angle1)
                print 'joint0_angle1 : ', joint0_angle1*2.1, ' deg : ', math.degrees(joint0_angle1)
                print 'joint1_angle1 : ', joint1_angle1*2.1, ' deg : ', math.degrees(joint1_angle1)
                print 'joint2_angle1 : ', joint2_angle1, ' deg : ', math.degrees(joint2_angle1)
                joint0_angle2 = -1*math.acos((x0*x0+y*y+l1*l1-l2*l2) / (2*l1*math.sqrt(x0*x0+y*y))) + math.atan(y/x0)
                joint1_angle2 = math.atan((y-l1*math.sin(joint0_angle2))/(x-l1*math.cos(joint0_angle2)))-joint0_angle2
                joint2_angle2 = -1*(joint0_angle2 + joint1_angle2)
                print 'joint0_angle2 : ', joint0_angle2*2.1, ' deg : ', math.degrees(joint0_angle2)
                print 'joint1_angle2 : ', joint1_angle2*2.1, ' deg : ', math.degrees(joint1_angle2)
                print 'joint2_angle2 : ', joint2_angle2, ' deg : ', math.degrees(joint2_angle2)
            else:
                print 'data1 : ', data1
                print 'data2 : ', data2
            '''
            thread_shoulder = threading.Thread(target=self.shoulderPub, args=(joint0_angle2*2.1,))
            thread_elbow = threading.Thread(target=self.elbowPub, args=(joint1_angle2*2.1,))
            thread_wrist = threading.Thread(target=self.wristPub, args=(joint2_angle2,))
            '''
            thread_shoulder = threading.Thread(target=self.shoulderPub, args=(-1.07115820647,))
            thread_elbow = threading.Thread(target=self.elbowPub, args=(2.3719040394,))
            thread_wrist = threading.Thread(target=self.wristPub, args=(-0.619402777586,))
            thread_shoulder.start()
            thread_elbow.start()
            thread_wrist.start()
            self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
            rospy.sleep(3.0)
            move_range = (0.17+obj_cog.x+0.15-(x0+0.2))*2.6
            self.MoveBase(move_range*0.7)
            rospy.sleep(0.4)
            self.MoveBase(move_range*0.3)
            grasp_flg = self.endeffectorPub(True)
            rospy.sleep(2.0)
            self.shoulderPub(0.7)
            self.MoveBase(-0.6)
            arm_change_cmd.data = 'carry'
            self.ChangePoseReqCB(arm_change_cmd)
            rospy.sleep(4.0)
            if grasp_flg :
                grasp_flg = self.m4_error > 0.03
            print grasp_flg
            if grasp_flg :
                self.grasp_res_pub.publish(grasp_flg)
            else:
                retry_cmd = String()
                retry_cmd.data = 'retry'
                self.retry_pub.publish(retry_cmd)
                return
            self.MoveBase(-0.4)
            print "finish"

    def shoulderPub(self, m01):
        m0_p = self.M0_ORIGIN_ANGLE - m01
        m1_p = self.M1_ORIGIN_ANGLE + m01
        self.m0_pub.publish(m0_p)
        self.m1_pub.publish(m1_p)
        rospy.sleep(0.1)
        while self.m0_is_moving or self.m1_is_moving:
            pass
        rospy.sleep(0.1)
        if self.m1_error > 0.1 or self.m0_error > 0.1:
            self.m1_pub.publish(self.m1_current_pos)
            self.m0_pub.publish(self.m0_current_pos)

    def elbowPub(self, m2):
        m2 *= -1
        m2_p = self.M2_ORIGIN_ANGLE + m2
        self.m2_pub.publish(m2_p)
        rospy.sleep(0.1)
        while self.m2_is_moving:
            pass
        if self.m2_error > 0.1:
            self.m2_pub.publish(self.m2_current_pos)

    def wristPub(self, m3):
        m3_p = self.M3_ORIGIN_ANGLE + m3
        self.m3_pub.publish(m3_p)
        rospy.sleep(0.1)
        while self.m3_is_moving:
            pass
        if self.m3_error > 0.1:
            self.m3_pub.publish(self.m3_current_pos)

    def endeffectorPub(self,req):
        angle = self.M4_ORIGIN_ANGLE
        self.m4_pub.publish(angle)
        rospy.sleep(0.1)
        while self.m4_is_moving and rospy.is_shutdown:
            pass
        rospy.sleep(0.1)
        grasp_flg = True
        while self.m4_error <= 0.08 and rospy.is_shutdown:
            angle -= 0.05
            self.m4_pub.publish(angle)
            rospy.sleep(0.09)
            while self.m4_velocity >= 2.0:
                pass
            if angle < -0.4:
                grasp_flg = False
                break;
        rospy.sleep(0.1)
        self.m4_pub.publish(self.m4_current_pos -  0.06)
        return grasp_flg
            
    def M0StateCB(self,state):
        self.m0_angle = state.current_pos
        self.m0_current_pos = state.current_pos
        self.m0_error = abs(state.error)
        self.m0_is_moving = state.is_moving
        #print "diff = ",self.m0_angle+self.m1_angle
    def M1StateCB(self,state):
        self.m1_angle = state.current_pos
        self.m1_current_pos = state.current_pos
        self.m1_error = abs(state.error)
        self.m1_is_moving = state.is_moving

    def M2StateCB(self,state):
        self.m2_current_pos = state.current_pos
        self.m2_error = abs(state.error)
        self.m2_is_moving = state.is_moving

    def M3StateCB(self,state):
        self.m3_current_pos = state.current_pos
        self.m3_error = abs(state.error)
        self.m3_is_moving = state.is_moving
        
    def M4StateCB(self,state):
        self.m4_current_pos = state.current_pos
        self.m4_error = abs(state.error)
        self.m4_is_moving = state.is_moving
        self.m4_velocity = abs(state.velocity)
    
    def LaserCB(self,laser_scan):
        self.front_laser_dist = laser_scan.ranges[360]
        #print self.front_laser_dist

if __name__ == '__main__':
    rospy.init_node('Manipulation',anonymous=True)
    manipulation = Manipulation()
    rospy.spin()
