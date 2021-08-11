#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import numpy as np
import numpy.linalg as lin
import math as m
from tf import TransformBroadcaster
from rospy import Time
import message_filters

def rad2deg(rad):
    return rad / np.pi * 180

def deg2rad(deg):
    return deg / 180 * np.pi

class System(object):
    def __init__(self):
        self._imu_sub = rospy.Subscriber("mavros/imu/data_raw", Imu, self.CallBack)
        self._mag_sub = rospy.Subscriber("mavros/imu/mag", MagneticField, self.MagCallBack)
        self._euler_sub = rospy.Subscriber("accel_to_euler", Vector3, self.EulerCallBack)
        #self._kalman_pub = rospy.Publisher("orientation", Vector3, queue_size = 10)
        self._kalman_pose_pub = rospy.Publisher("fusion_orientation", PoseStamped, queue_size=10)

        self.dt = 0.02
        self._xhat = np.zeros((3,1))
        self.q = [1,1,1,1]

        ## KALMAN FILTER VARIABLES
        # state covariance (initial is implicit)
        self.P = np.array([[100.0, 0, 0],
                        [0, 100.0, 0],
                        [0, 0, 100.0]])
        # process noise (assumed randomly , can be refined using statistical analysis of gyro)
        self.Q = np.array([[0.01, 0, 0],
                        [0, 0.01, 0],
                        [0, 0, 0.01]])

        # measurement noise (assumed randomly, can be refined using statistical analysis of accel and magnetometer)
        self.R = np.array([[0.1, 0, 0],
                        [0, 0.1, 0],
                        [0, 0, 0.1]])

        self._gyro = Vector3()
        self._acc = Vector3()
        self._mag = Vector3()
        self._pose = PoseStamped()

    def CallBack(self, msg) :
        self._gyro.x = msg.angular_velocity.x
        self._gyro.y = msg.angular_velocity.y
        self._gyro.z = msg.angular_velocity.z

        self._acc.x = msg.linear_acceleration.x
        self._acc.y = msg.linear_acceleration.y
        self._acc.z = msg.linear_acceleration.z

        ## extract data
        # ax, ay, az = msg_accel.data
        # mx, my, mz = msg_mag.data
        # wx, wy, wz = msg_gyro.data

        ### ACCELEROMETER AND MAGNETOMETER CALCULATIONS ###
        ## normalize accelerometer data
        norm_accel =  m.sqrt(self._acc.x**2 + self._acc.y**2 + self._acc.z**2)
        if norm_accel != 0 :
            self._acc.x = self._acc.x / norm_accel
            self._acc.y = self._acc.y / norm_accel
            self._acc.z = self._acc.z / norm_accel

        ## normalize the magnetometer data
        norm_mag =  m.sqrt(self._mag.z**2 + self._mag.y**2 + self._mag.z**2)
        if norm_mag != 0 :
            self._mag.x = self._mag.x / norm_mag
            self._mag.y = self._mag.y / norm_mag
            self._mag.z = self._mag.z / norm_mag

        ## get the roll and pitch
        rollA = self._xhat[0][0]
        pitchA = self._xhat[1][0]

        ## compensate for yaw using magnetometer
        Mx = self._mag.x * m.cos(pitchA) + self._mag.z * m.sin(pitchA)
        My = self._mag.x * m.sin(rollA) * m.sin(pitchA) + self._mag.y * m.cos(rollA) - self._mag.z * m.sin(rollA) * m.cos(pitchA)

        yawM = m.atan2(-My, Mx)

        ## rollA, pitchA, yawM are the measurements

        ## PREDICTION IF NEEDED
        
        # carry out prediction step
        # we need to calculate time_elapsed
        # secs = Time.now().secs
        # nsecs = Time.now().nsecs
        # cur_time = secs + nsecs * 10 ** (-9)
        # time_elapsed = cur_time - prev_time
        # prev_time = cur_time

        # integrate using eulers integration method to find the estimates and covariance
        # q, P = integrateTillT(q, dt, time_elapsed, wx, wy, wz, P)
        # make the state prediction
        current_gyro_quat = quaternion_from_euler(self._gyro.x * self.dt, self._gyro.y * self.dt, self._gyro.z * self.dt)
        self.q = quaternion_multiply(current_gyro_quat, self.q)
        # make the covariance prediction
        self.P += self.Q
        # extract angles
        rollF, pitchF, yawF = euler_from_quaternion(self.q)

        # compute kalman gain
        K = np.dot(self.P, np.linalg.inv(self.P + self.R))

        # carry out correction step
        meas = np.array([[rollA],[pitchA],[yawM]])
        state = np.array([[rollF],[pitchF],[yawF]])

        state = state + np.dot(K, meas - state)
        state = [float(val) for val in state]
        rollF, pitchF, yawF = state

        # update the quaternion
        q = quaternion_from_euler(rollF, pitchF, yawF)
        self._pose.pose.position.x = 0
        self._pose.pose.position.y = 0
        self._pose.pose.position.z = 0
        self._pose.pose.orientation.x = q[0]
        self._pose.pose.orientation.y = q[1]
        self._pose.pose.orientation.z = q[2]
        self._pose.pose.orientation.w = -q[3]

        self._pose.header.stamp = rospy.Time.now()
        self._pose.header.frame_id = '/base_link'

        # update covariance
        self.P = np.dot((np.eye(3) - K), self.P)

    def MagCallBack(self, msg) :
        self._mag.x = msg.magnetic_field.x
        self._mag.y = msg.magnetic_field.y
        self._mag.z = msg.magnetic_field.z

    def EulerCallBack(self, msg) :
        phi = deg2rad(msg.x) #roll (radian)
        theta = deg2rad(msg.y) #pitch
        psi = deg2rad(msg.z) #yaw
        self._xhat[0][0] = phi
        self._xhat[1][0] = theta
        self._xhat[2][0] = psi

    def Publish(self) :
        self._kalman_pose_pub.publish(self._pose)

if __name__ == '__main__' :
    rospy.init_node("fusion_orientation")
    rate = rospy.Rate(50)
    test = System()
    while not rospy.is_shutdown() :
        test.Publish()
        rate.sleep()
    rospy.spin()