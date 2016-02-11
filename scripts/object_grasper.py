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
#from sensor_msgs.msg import LaserScan
from experiment_motor import Experiment

class ObjectGrasper(Experiment):
    def __init__(self):
        super(ObjectGrasper,self).__init__()
        self.xyz_centroid_sub = rospy.Subscriber('/object/xyz_centroid',Point,self.main)
        self.changing_pose_req_sub = rospy.Subscriber('/arm/changing_pose_req',String,self.changePoseReqCB)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size = 1)
        self.grasp_res_pub = rospy.Publisher('/object/grasp_res',Bool,queue_size = 1)
        self.changing_pose_res_pub = rospy.Publisher('/arm/changing_pose_res',Bool,queue_size = 1)
        self.retry_pub = rospy.Publisher('/object/grasp_req',String,queue_size = 1)
        #self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.laserCB)

        # instance variables
        self.search_count = 0
        self.grasp_count = 0
        #self.front_laser_dist = 0.00

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
            self.moveBase(0.6)
            '''
            move_range = (self.front_laser_dist-0.75)*3.0
            self.moveBase(move_range * 0.6)
            rospy.sleep(0.4)
            self.moveBase(move_range * 0.4)
            '''
            rospy.sleep(0.4)
            angle = 0.0
            elbow_current = self.m2_current_pos
            wrist_current = self.m3_current_pos
            wrist_error = self.m3_error + 0.01
            #while self.m3_error <= -0.015 and not rospy.is_shutdown():
            while self.m3_error - wrist_error < 0.015 and not rospy.is_shutdown():
                if (angle*100) % 2:
                    wrist_error = self.m3_error
                angle += 0.03
                elbow_angle = elbow_current + angle*2
                wrist_angle = wrist_current + angle
                #self.armController(shoulder_param, elbow_param, wrist_param)
                self.m2_pub.publish(elbow_angle)
                rospy.sleep(0.1)
                self.m3_pub.publish(wrist_angle)
                rospy.sleep(0.05)
                while (self.m2_is_moving or self.m3_is_moving) and not rospy.is_shutdown():
                    pass
                if angle > 1.5:
                    break
                rospy.sleep(0.1)
            self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
            rospy.sleep(1.0)
            self.moveBase(-0.3)
            self.shoulderPub(shoulder_param+0.5)
            self.moveBase(-0.8)
            arm_change_cmd = String()
            arm_change_cmd.data = 'carry'
            self.changePoseReqCB(arm_change_cmd)
            self.changing_pose_res_pub.publish(True)
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

    def localizeObject(self,obj_cog):
        print '\n-- Localize Object --'
        arm_change_cmd = String()
        arm_change_cmd.data = 'carry'
        self.changePoseReqCB(arm_change_cmd)
        self.m4_pub.publish(self.M4_ORIGIN_ANGLE)
        obj_cog.y += 0.08 # calibrate RealSenseCamera d435
        #obj_cog.z -= 0.015
        print obj_cog
        if math.isnan(obj_cog.x):
            if self.search_count == 3:
                self.grasp_res_pub.publish(False)
                return False
            else:
                self.search_count += 1
                move_range = -1*((self.search_count%4)/2)*1.2+0.6
                self.moveBase(move_range)
                self.retry_pub.publish('retry')
                return False
        self.search_count = 0
        obj_angle = math.atan2(obj_cog.y,obj_cog.x)
        print 'obj_angle : ', obj_angle
        if obj_angle < -0.05 or 0.05 <obj_angle:
            print 'There is not object in front.'
            #move kobuki
            #revolve to object
            cmd = Twist()
            cmd.linear.x = 0
            cmd.angular.z = obj_angle * 4.0
            if 0 < abs(cmd.angular.z)  and abs(cmd.angular.z) < 0.85:
                cmd.angular.z = int(cmd.angular.z/abs(cmd.angular.z))*0.85
            print 'cmd.angular.z : ', cmd.angular.z
            self.cmd_vel_pub.publish(cmd)
            time.sleep(3)
            cmd.angular.z = 0
            self.cmd_vel_pub.publish(cmd)
            self.retry_pub.publish('retry')
            print 'Finish roll'
            return False
        elif obj_cog.x < 0.6 or obj_cog.x > 0.7:
            move_range = (obj_cog.x-0.65)*2.0
            if abs(move_range) < 0.5:
                move_range = int(move_range/abs(move_range))*0.5
            print 'move_range : ', move_range
            self.moveBase(move_range)
            self.retry_pub.publish('retry')
            print 'Finish move'
            return False
        else :
            return True

    def graspObject(self, obj_cog):
        print '-- Grasp Object --'
        self.moveBase(-0.6)
        y = obj_cog.z
        x = (y-0.75)/10+0.5 #0.5
        s_a, e_a, w_a = self.inverseKinematics(x, y)
        self.armController(s_a, e_a, w_a)
        '''
        ### 決め打ち用
        shoulder_angle = -1.07115820647
        elbow_angle = 2.3719040394
        wrist_angle = -0.619402777586
        self.armController(shoulder_angle, elbow_angle, wrist_angle)
        ###
        '''
        rospy.sleep(3.0)
        move_range = (0.17+obj_cog.x+0.15-(x+0.2))*3.1
        self.moveBase(move_range*0.7)
        rospy.sleep(0.3)
        self.moveBase(move_range*0.3)
        grasp_flg = self.endeffectorPub(True)
        rospy.sleep(2.0)
        self.shoulderPub(-0.5)
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
        print 'Finish grasp'
        return

    def inverseKinematics(self, x, y):
        l0 = 0.75# Height from ground to shoulder(metre)
        l1 = 0.24# Length from shoulder to elbow(metre)
        l2 = 0.20# Length from elbow to wrist(metre)
        l3 = 0.15# Length of end effector(metre)
        x -= l3
        y -= l0
        data1 =  x*x+y*y+l1*l1-l2*l2
        data2 =  2*l1*math.sqrt(x*x+y*y)
        if data1 > data2:
            print 'I can not move arm.'
            self.retry_pub.publish('retry')
            return
        shoulder_angle = -1*math.acos((x*x+y*y+l1*l1-l2*l2) / (2*l1*math.sqrt(x*x+y*y))) + math.atan(y/x)# -1倍の有無で別解
        elbow_angle = math.atan((y-l1*math.sin(shoulder_angle))/(x-l1*math.cos(shoulder_angle)))-shoulder_angle
        wrist_angle = -1*(shoulder_angle + elbow_angle)
        shoulder_angle *= 2.1
        elbow_angle *= 2.1
        print 'shoulder_angle : ', shoulder_angle, ' deg : ', math.degrees(shoulder_angle)
        print 'elbow_angle    : ', elbow_angle, ' deg : ', math.degrees(elbow_angle)
        print 'wrist_angle    : ', wrist_angle, ' deg : ', math.degrees(wrist_angle)
        #self.armController(shoulder_angle, elbow_angle, wrist_angle)
        return shoulder_angle, elbow_angle, wrist_angle
        
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

    def main(self,obj_cog):
        localize_flg = self.localizeObject(obj_cog)
        if localize_flg:
            self.graspObject(obj_cog)

if __name__ == '__main__':
    rospy.init_node('Object_Grasper')
    grasper = ObjectGrasper()
    rospy.spin()
