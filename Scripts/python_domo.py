#!/usr/bin/python2.7
# coding=UTF-8
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String

import time
# from pyquaternion import Quaternion
import math
import threading
import numpy as np
import matplotlib.pyplot as plt
import datetime

uav_ID = 1


class Px4Controller:

    def __init__(self):

        self.imu = None
        self.gps = None
        self.current_state = None
        self.current_heading = None
        self.takeoff_height = 2
        self.local_enu_position = None

        self.cur_target_pose = None
        self.global_target = None

        self.received_new_task = False
        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"
        self.state = None

        # 飞机的时间
        self.time = 0

        # 飞机的初始位置
        self.init_x = 0
        self.init_y = 3
        self.init_z = 0

        # 记录每个无人机的全局位置
        self.poseuav = [0, 1, 2, 3]
        for i in range(4):
            self.poseuav[i] = PoseStamped()

        # 记录自己的位置
        self.local_pose = None

        # 设置无人机的目标状态
        self.TargetPose = PoseStamped()

        '''
        ros subscribers
        '''
        self.uav0_local_pose_sub = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped,
                                                    self.uav0_local_pose_callback)
        self.uav1_local_pose_sub = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped,
                                                    self.uav1_local_pose_callback)
        self.uav2_local_pose_sub = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped,
                                                    self.uav2_local_pose_callback)
        self.uav3_local_pose_sub = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped,
                                                    self.uav3_local_pose_callback)
        self.mavros_sub = rospy.Subscriber("uav1/mavros/state", State, self.mavros_state_callback)

        '''
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('/uav1/mavros/setpoint_position/local',PoseStamped, queue_size=1)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)

        print("Px4 Controller Initialized!")

    def uav0_local_pose_callback(self, msg):
        self.poseuav[0].pose.position = msg.pose.position
        self.poseuav[1].pose.position.y = msg.pose.position.y + 3

    def uav1_local_pose_callback(self, msg):
        self.local_pose = msg
        self.time = msg.header.stamp.secs
        self.poseuav[1].pose.position = msg.pose.position
        self.poseuav[1].pose.position.y = msg.pose.position.y + 1

    def uav2_local_pose_callback(self, msg):
        self.poseuav[2].pose.position = msg.pose.position
        self.poseuav[2].pose.position.y = msg.pose.position.y - 1

    def uav3_local_pose_callback(self, msg):
        self.poseuav[3].pose.position = msg.pose.position
        self.poseuav[3].pose.position.y = msg.pose.position.y - 3

    def start(self):
        rospy.init_node("offboard_node")
        self.cur_target_pose = self.construct_target(0, 0, self.takeoff_height, True)
        # print ("self.cur_target_pose:", self.cur_target_pose, type(self.cur_target_pose))

        for i in range(40):
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            self.local_target_pub.publish(self.cur_target_pose)
            time.sleep(0.2)
        print ("the time finished")

        if self.takeoff_detection():
            print("Vehicle Took Off!")

        else:
            print("Vehicle Took Off Failed!")
            return

        '''
        main ROS thread
        '''
        while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
            print ("----the time finished")
            # time1 = datetime.datetime.now()
            if self.local_pose.pose.position.z < 2.2 and self.local_pose.pose.position.z > 1.8:
                self.TargetPose.pose.position.x = 3*np.cos(self.time+np.pi/2)
                self.TargetPose.pose.position.y = 3*np.sin(self.time+np.pi/2) - 3
                self.TargetPose.pose.position.z = 2
                self.local_target_pub.publish(self.TargetPose)

            else:
                self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x-self.init_x, self.local_pose.pose.position.y-self.init_y, self.takeoff_height, True)

                self.local_target_pub.publish(self.cur_target_pose)
                print ('z position is out of range!')

            if (self.state is "LAND") and (self.local_pose.pose.position.z < 0.15):

                if (self.disarm()):
                    print ("-------")

                    self.state = "DISARMED"

            time.sleep(0.01)

    def construct_target(self, x, y, z, yaw, yaw_rate=1):
        target_raw_pose = PoseStamped()
        target_raw_pose.header.stamp = rospy.Time.now()

        # target_raw_pose.coordinate_frame = 9

        target_raw_pose.pose.position.x = x
        target_raw_pose.pose.position.y = y
        target_raw_pose.pose.position.z = z

        # target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
        #                             + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
        #                             + PositionTarget.FORCE

        # target_raw_pose.yaw = yaw
        # target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose

    '''
    cur_p : poseStamped
    target_p: positionTarget
    '''

    def position_distance(self, cur_p, target_p, threshold=0.1):
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x)
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)

        if (delta_x + delta_y + delta_z < threshold):
            return True
        else:
            return False

    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    def gps_callback(self, msg):
        self.gps = msg

    def FLU2ENU(self, msg):

        FLU_x = msg.pose.position.x * math.cos(self.current_heading) - msg.pose.position.y * math.sin(
            self.current_heading)
        FLU_y = msg.pose.position.x * math.sin(self.current_heading) + msg.pose.position.y * math.cos(
            self.current_heading)
        FLU_z = msg.pose.position.z

        return FLU_x, FLU_y, FLU_z

    def set_target_position_callback(self, msg):
        print("Received New Position Task!")

        if msg.header.frame_id == 'base_link':
            '''
            BODY_FLU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self.frame = "BODY"

            print("body FLU frame")

            ENU_X, ENU_Y, ENU_Z = self.FLU2ENU(msg)

            ENU_X = ENU_X + self.local_pose.pose.position.x
            ENU_Y = ENU_Y + self.local_pose.pose.position.y
            ENU_Z = ENU_Z + self.local_pose.pose.position.z

            self.cur_target_pose = self.construct_target(ENU_X,
                                                         ENU_Y,
                                                         ENU_Z,
                                                         self.current_heading)


        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X

            self.frame = "LOCAL_ENU"
            print("local ENU frame")

            self.cur_target_pose = self.construct_target(msg.pose.position.x,
                                                         msg.pose.position.y,
                                                         msg.pose.position.z,
                                                         self.current_heading)

    '''
     Receive A Custom Activity
     '''

    def custom_activity_callback(self, msg):

        print("Received Custom Activity:", msg.data)

        if msg.data == "LAND":
            print("LANDING!")
            self.state = "LAND"
            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                         self.local_pose.pose.position.y,
                                                         0.1,
                                                         self.current_heading)

        if msg.data == "HOVER":
            print("HOVERING!")
            self.state = "HOVER"
            self.hover()

        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")

    def set_target_yaw_callback(self, msg):
        print("Received New Yaw Task!")

        yaw_deg = msg.data * math.pi / 180.0
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     yaw_deg)

    '''
    return yaw from current IMU
    '''

    # def q2yaw(self, q):
    #     if isinstance(q, Quaternion):
    #         rotate_z_rad = q.yaw_pitch_roll[0]
    #     else:
    #         q_ = Quaternion(q.w, q.x, q.y, q.z)
    #         rotate_z_rad = q_.yaw_pitch_roll[0]
    #
    #     return rotate_z_rad

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False

    def hover(self):

        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     self.current_heading)

    def takeoff_detection(self):
        if self.local_pose.pose.position.z > 0.1 and self.offboard_state and self.arm_state:
            return True
        else:
            return False


if __name__ == '__main__':
    con = Px4Controller()
    con.start()
