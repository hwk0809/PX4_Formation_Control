#!/usr/bin/python2.7
# coding=UTF-8
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point, Vector3Stamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String

import time
# from pyquaternion import Quaternion
import math
import threading
import numpy as np
import matplotlib.pyplot as plt
import datetime

uav_ID = 3
num = 4  # 无人机个数
Height = 2  # 起飞高度


# 追踪时变队形
class FormationControl:
    def __init__(self):
        self.id = uav_ID  # 无人机id
        self.num = 4  # 无人机个数
        self.time = 0  # 初始时间
        self.t0 = 0  # 开始执行任务的时间

        # 初始在地面坐标（x,y）
        self.init = list(range(0, self.num))
        self.init[0] = [3, 0]
        self.init[1] = [0, 3]
        self.init[2] = [-3, 0]
        self.init[3] = [0, -3]

        # 队形信息
        self.TargetPosition = list(range(0, self.num))
        self.TargetVelocity = list(range(0, self.num))
        self.TargetVelocityDot = list(range(0, self.num))
        for i in range(self.num):
            self.TargetPosition[i] = Point()
            self.TargetVelocity[i] = Twist()
            self.TargetVelocityDot[i] = Twist()

        # 飞机获取信息的数据预定义
        self.local_pose = PoseStamped()  # 此飞机坐标
        self.poseuav = list(range(0, self.num))
        self.Velocity_uav = list(range(0, self.num))
        for i in range(self.num):
            self.poseuav[i] = Pose()  # all the uav position
            self.Velocity_uav[i] = Twist()  # all the uav velocity
        self.ksi = np.array([0] * 24, dtype=np.float64).reshape((-1, 1))
        self.ksi_initial()

        '''
        ros subscribers
        '''
        # local position
        self.uav0_local_pose_sub = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped,
                                                    self.uav0_local_pose_callback)
        self.uav1_local_pose_sub = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped,
                                                    self.uav1_local_pose_callback)
        self.uav2_local_pose_sub = rospy.Subscriber("/uav2/mavros/local_position/pose", PoseStamped,
                                                    self.uav2_local_pose_callback)
        self.uav3_local_pose_sub = rospy.Subscriber("/uav3/mavros/local_position/pose", PoseStamped,
                                                    self.uav3_local_pose_callback)
        # local velocity
        self.uav0_local_velocity_sub = rospy.Subscriber("/uav0/mavros/local_position/velocity",
                                                        Twist, self.uav0_local_velocity_callback)
        self.uav1_local_velocity_sub = rospy.Subscriber("/uav1/mavros/local_position/velocity",
                                                        Twist, self.uav1_local_velocity_callback)
        self.uav2_local_velocity_sub = rospy.Subscriber("/uav2/mavros/local_position/velocity",
                                                        Twist, self.uav2_local_velocity_callback)
        self.uav3_local_velocity_sub = rospy.Subscriber("/uav3/mavros/local_position/velocity",
                                                        Twist, self.uav3_local_velocity_callback)
        print("Formation Controller Initialized!")

    '''
    执行函数
    '''
    def ksi_initial(self):
        for i in range(self.num):
            self.ksi[6 * i][0] = self.init[i][0]
            self.ksi[6 * i + 2][0] = self.init[i][1]
            self.ksi[6 * i + 4][0] = Height
        print ("init ksi")
        print self.ksi

    def init_t(self, t0):
        self.t0 = t0

    def formation_circle(self):
        r = 3  # 这里定义的是四个无人机画圆的r
        w = 0.005  # 绕圆角速度
        height = 2  # 高度
        for i in range(self.num):
            self.TargetPosition[i].x = r * np.sin(w * (self.time - self.t0) + np.pi / 2 * i)
            self.TargetPosition[i].y = r * np.cos(w * (self.time - self.t0) + np.pi / 2 * i)
            self.TargetPosition[i].z = height
            self.TargetVelocity[i].linear.x = w * r * np.cos(w * (self.time - self.t0) + np.pi / 2 * i)
            self.TargetVelocity[i].linear.y = -w * r * np.sin(w * (self.time - self.t0) + np.pi / 2 * i)
            self.TargetVelocity[i].linear.z = 0
            self.TargetVelocityDot[i].linear.x = -r * w * w * np.sin(w * (self.time - self.t0) + np.pi / 2 * i)
            self.TargetVelocityDot[i].linear.y = -r * w * w * np.cos(w * (self.time - self.t0) + np.pi / 2 * i)
            self.TargetVelocityDot[i].linear.z = 0

    def controller(self):
        L = np.array([[1, -1, 0, 0], [-1, 2, - 1, 0], [0, - 1, 2, - 1], [0, 0, - 1, 1]])
        H = np.array([0] * 24,dtype=np.float64).reshape((-1, 1))
        Hv = np.array([0] * 12,dtype=np.float64).reshape((-1, 1))
        B1 = np.kron(np.eye(3), np.array([1, 0]).reshape((-1, 1)))
        B2 = np.kron(np.eye(3), np.array([0, 1]).reshape((-1, 1)))
        K1 = np.kron(np.eye(3), np.array([-1, -0.8]).reshape((1, -1)))
        K2 = np.kron(np.eye(3), np.array([0.3535, 0.6582]).reshape((1, -1)))
        P = np.kron(np.eye(3), np.array([[1.4219, 0.4242], [0.4142, 0.7771]]))
        for i in range(self.num):
            self.ksi[6 * i][0] = self.poseuav[i].position.x
            self.ksi[6 * i + 1][0] = self.Velocity_uav[i].linear.x
            self.ksi[6 * i + 2][0] = self.poseuav[i].position.y
            self.ksi[6 * i + 3][0] = self.Velocity_uav[i].linear.y
            self.ksi[6 * i + 4][0] = self.poseuav[i].position.z
            self.ksi[6 * i + 5][0] = self.Velocity_uav[i].linear.z

            H[6 * i][0] = self.TargetPosition[i].x
            H[6 * i + 1][0] = self.TargetVelocity[i].linear.x
            H[6 * i + 2][0] = self.TargetPosition[i].y
            H[6 * i + 3][0] = self.TargetVelocity[i].linear.y
            H[6 * i + 4][0] = self.TargetPosition[i].z
            H[6 * i + 5][0] = self.TargetVelocity[i].linear.z

            Hv[3 * i][0] = self.TargetVelocityDot[i].linear.x
            Hv[3 * i + 1][0] = self.TargetVelocityDot[i].linear.y
            Hv[3 * i + 2][0] = self.TargetVelocityDot[i].linear.z

        ksi_temp = np.kron(np.eye(self.num), np.matmul(B2, K1) + np.matmul(B1, B2.T)) - np.kron(L, np.matmul(B2, K2))
        H_temp = np.kron(np.eye(self.num), np.matmul(B2, K1)) - np.kron(L, np.matmul(B2, K2))
        Hv_temp = np.kron(np.eye(self.num), B2)
        U = np.matmul(ksi_temp, self.ksi) - np.matmul(H_temp, H) + np.matmul(Hv_temp, Hv)
        U_control_v = [U[6 * self.id][0], U[6 * self.id + 2][0], U[6 * self.id + 4][0]]
        U_control_a = [U[6 * self.id + 1][0], U[6 * self.id + 3][0], U[6 * self.id + 5][0]]
        print ("cal ksi")
        print (self.ksi)
        print ("U_control_a")
        print U
        return U_control_a, U_control_v

    '''
    回调函数
    '''

    # position
    def uav0_local_pose_callback(self, msg):
        self.local_pose = msg
        self.time = msg.header.stamp.secs
        self.poseuav[0].position = msg.pose.position
        self.poseuav[0].position.x = msg.pose.position.x + self.init[0][0]
        self.poseuav[0].position.y = msg.pose.position.y + self.init[0][1]

    def uav1_local_pose_callback(self, msg):
        self.poseuav[1].position = msg.pose.position
        self.poseuav[1].position.x = msg.pose.position.x + self.init[1][0]
        self.poseuav[1].position.y = msg.pose.position.y + self.init[1][1]


    def uav2_local_pose_callback(self, msg):
        self.poseuav[2].position = msg.pose.position
        self.poseuav[2].position.x = msg.pose.position.x + self.init[2][0]
        self.poseuav[2].position.y = msg.pose.position.y + self.init[2][1]

    def uav3_local_pose_callback(self, msg):
        self.poseuav[3].position = msg.pose.position
        self.poseuav[3].position.x = msg.pose.position.x + self.init[3][0]
        self.poseuav[3].position.y = msg.pose.position.y + self.init[3][1]

    # velocity
    def uav0_local_velocity_callback(self, msg):
        self.Velocity_uav[0].linear = msg.linear

    def uav1_local_velocity_callback(self, msg):
        self.Velocity_uav[1].linear = msg.linear

    def uav2_local_velocity_callback(self, msg):
        self.Velocity_uav[2].linear = msg.linear

    def uav3_local_velocity_callback(self, msg):
        self.Velocity_uav[3].linear = msg.linear


# PX4 控制器
class Px4Controller:
    def __init__(self):

        self.imu = None
        self.gps = None
        self.current_state = None
        self.current_heading = None
        self.takeoff_height = Height
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
        self.init = list(range(0, num))
        self.init[0] = [3, 0]
        self.init[1] = [0, 3]
        self.init[2] = [-3, 0]
        self.init[3] = [0, -3]

        # 记录自己的位置
        self.local_pose = None

        # 设置无人机的目标状态
        self.TargetPose = PoseStamped()  # 目标位置
        self.TargetVelocity = Twist()  # 速度
        self.TargetAcc = Vector3Stamped()  # 加速度

        '''
        ros subscribers
        '''
        self.uav_local_pose_sub = rospy.Subscriber("/uav" + str(uav_ID) + "/mavros/local_position/pose", PoseStamped,
                                                   self.uav_local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/uav" + str(uav_ID) + "/mavros/state", State, self.mavros_state_callback)

        '''
        ros publishers
        '''
        self.local_target_p_pub = rospy.Publisher("/uav" + str(uav_ID) + "/mavros/setpoint_position/local", PoseStamped,
                                                  queue_size=1)
        # self.local_target_v_pub = rospy.Publisher("/uav" + str(uav_ID) + "/mavros/setpoint_velocity/cmd_vel_unstamped",
        #                                           Twist, queue_size=1)
        self.local_target_a_pub = rospy.Publisher("/uav" + str(uav_ID) + "/mavros/setpoint_accel/accel", Vector3Stamped,
                                                  queue_size=1)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy("/uav" + str(uav_ID) + "/mavros/cmd/arming", CommandBool)
        self.flightModeService = rospy.ServiceProxy("/uav" + str(uav_ID) + "/mavros/set_mode", SetMode)
        print("Px4 Controller Initialized!")

    def uav_local_pose_callback(self, msg):
        self.local_pose = msg
        self.time = msg.header.stamp.secs

    def start(self):
        rospy.init_node("offboard_node")
        self.cur_target_pose = self.construct_target(0, 0, self.takeoff_height,True)  # construct desired height data

        Controller = FormationControl()  # initial

        for i in range(40):
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            self.local_target_p_pub.publish(self.cur_target_pose)   # take off
            time.sleep(0.2)
        print ("the time finished")

        if self.takeoff_detection():
            print("Vehicle Took Off!")

        else:
            print("Vehicle Took Off Failed!")
            return

        t0 = rospy.Time.now().secs
        Controller.init_t(t0)

        '''
        main controller
        '''
        while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
            if self.local_pose.pose.position.z < 2.2 and self.local_pose.pose.position.z > 1.8:
                print("------------------------------------------------------------")
                print ("poseuav")
                print Controller.poseuav
                print ("ksi")
                print Controller.ksi
                Controller.formation_circle()  # construct formation
                Control_acc, Control_velocity = Controller.controller()  # get the control input
                self.TargetVelocity.linear.x = Control_velocity[0]
                self.TargetVelocity.linear.y = Control_velocity[1]
                self.TargetVelocity.linear.z = Control_velocity[2]   # velocity
                self.TargetAcc.vector.x = Control_acc[0]
                self.TargetAcc.vector.y = Control_acc[1]
                self.TargetAcc.vector.z = Control_acc[2]            # accel
                # self.local_target_v_pub.publish(self.TargetVelocity)
                self.local_target_a_pub.publish(self.TargetAcc)
            else:
                self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.takeoff_height, True)
                self.local_target_p_pub.publish(self.cur_target_pose)  # 超出z范围 则先调整z

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
