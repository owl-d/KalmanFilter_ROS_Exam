#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import numpy as np
import math
import sys
#from tf import TransformBroadcaster
#from rospy import Time

class Algorithm(object):
    def __init__(self):
        self._imu_sub = rospy.Subscriber("mavros/imu/data_raw", Imu, self.CallBack)
        self._mag_sub = rospy.Subscriber("mavros/imu/mag", MagneticField, self.MagCallBack)
        self._pub = rospy.Publisher("orientation", PoseStamped, queue_size = 10)
        self._angular_velocity_pub = rospy.Publisher("angularVelocity", Vector3, queue_size = 10)
        self._pose = PoseStamped()

        self.gyroreadings = np.zeros((1,3))
        self.accelReadings = np.zeros((1,3))
        self.magReadings = np.zeros((1,3))
        self.gyroOffset = np.zeros((1,3))
        self.DecimationFactor = 1
        self.SampleRate = 50 #50hz
        self.q = np.zeros((1,4)) #post orientation #Quaternion
        self.q_hat = np.zeros((1,4)) #priori orientation #Quaternion
        self.linAccelprior = np.zeros((1,3))
        self.gAccel = np.zeros((1,3))
        self.g = np.zeros((1,3))
        self.mGyro = np.zeros((1,3))
        self.zg = np.zeros((1,3))
        self.z = np.zeros((1,6))
        self.mError = np.zeros((3,1))
        self.K = np.zeros((12,6))
        self.tf = False
        self.S = np.zeros((6,6))
        self.R = np.zeros((6,6))
        self.Pp = np.zeros((12,12)) #P-
        self.P = np.zeros((12,12))  #P+
        self.Q = np.zeros((12,12))
        self.x = np.zeros((12,1))
        self.inverse_term = np.ones((6,6))
        self.rPost = np.zeros((3,3)) #from q
        self.rPrior = np.zeros((3,3)) #from q_hat
        self.m = np.zeros((1,3)) #magnetic vector estimate
        self.angular_velocity = np.zeros((1,3))

        self.H = np.zeros((6, 12))
        self.H[0][6] = 1
        self.H[1][7] = 1
        self.H[2][8] = 1
        self.H[3][9] = -1
        self.H[4][10] = -1
        self.H[5][11] = -1

        self.AccelerometerNoise = 0.00019247 #need to search
        self.MagnetometerNoise = 1
        self.GyroscopeNoise = 9.1385e-5
        self.GyroscopedriftNoise = 3.0462e-13
        self.LinearAccelerationNoise = 0.0096236
        self.LinearAccelerationDecayFactor = 0.5
        self.MagneticDisturbanceNoise = 0.5
        self.MagneticDisturbanceDecayFactor = 0.5
        self.ExpectedMageticFieldStrength = 50
        self.OrientationFormat = Quaternion()

        self.InitialProcessNoise = np.zeros((12,12))
        self.InitialProcessNoise[0][0] = 0.000006092348396
        self.InitialProcessNoise[1][1] = 0.000006092348396
        self.InitialProcessNoise[2][2] = 0.000006092348396
        self.InitialProcessNoise[3][3] = 0.000076154354947
        self.InitialProcessNoise[4][4] = 0.000076154354947
        self.InitialProcessNoise[5][5] = 0.000076154354947
        self.InitialProcessNoise[6][6] = 0.009623610000000
        self.InitialProcessNoise[7][7] = 0.009623610000000
        self.InitialProcessNoise[8][8] = 0.009623610000000
        self.InitialProcessNoise[9][9] = 0.600000000000000
        self.InitialProcessNoise[10][10] = 0.600000000000000
        self.InitialProcessNoise[11][11] = 0.600000000000000
    
    def CallBack(self, msg):
        self.gyroreadings[0][0] = msg.angular_velocity.x
        self.gyroreadings[0][1] = msg.angular_velocity.y
        self.gyroreadings[0][2] = msg.angular_velocity.z

        self.accelReadings[0][0] = msg.linear_acceleration.x
        self.accelReadings[0][1] = msg.linear_acceleration.y
        self.accelReadings[0][2] = msg.linear_acceleration.z

        self.model()
        self.error_model()
        self.magnetometer_correct()
        self.kalman_equation()
        self.correct()
        self.compute_angular_velocity()
        self.update_magnetic_vector()
        self.Publish()

    def MagCallBack(self, msg):
        self.magReadings[0][0] = msg.magnetic_field.x
        self.magReadings[0][1] = msg.magnetic_field.y
        self.magReadings[0][2] = msg.magnetic_field.z

    def model(self):
        #Predict Orientation
        d_orien = (self.gyroreadings - self.gyroOffset) / self.SampleRate
        d_Q = quaternion_from_euler(d_orien[0][0], d_orien[0][1], d_orien[0][2])
        self.q_hat = self.q * np.prod(d_Q)

        #Estimate Gravity from Orientation
        self.rPrior[0][0] = 2*(self.q_hat[0][0]**2 + self.q_hat[0][1]**2) - 1
        self.rPrior[0][1] = 2*(self.q_hat[0][1]*self.q_hat[0][2] - self.q_hat[0][0]*self.q_hat[0][3])
        self.rPrior[0][2] = 2*(self.q_hat[0][1]*self.q_hat[0][3] + self.q_hat[0][0]*self.q_hat[0][2])
        self.rPrior[1][0] = 2*(self.q_hat[0][1]*self.q_hat[0][2] + self.q_hat[0][0]*self.q_hat[0][3])
        self.rPrior[1][1] = 2*(self.q_hat[0][0]**2 + self.q_hat[0][2]**2) - 1
        self.rPrior[1][2] = 2*(self.q_hat[0][2]*self.q_hat[0][3] - self.q_hat[0][0]*self.q_hat[0][1])
        self.rPrior[2][0] = 2*(self.q_hat[0][1]*self.q_hat[0][3] - self.q_hat[0][0]*self.q_hat[0][2])
        self.rPrior[2][1] = 2*(self.q_hat[0][2]*self.q_hat[0][3] + self.q_hat[0][0]*self.q_hat[0][1])
        self.rPrior[2][2] = 2*(self.q_hat[0][0]**2 + self.q_hat[0][3]**2) - 1

        self.g = self.rPrior[:,2].T #1X3
        self.g = self.g.reshape((1,3))

        #Estimate Gravity from Acceleration
        self.gAccel = self.accelReadings - self.linAccelprior

        #Estimate Earth's Magnetic Vector
        self.mGyro = (self.rPrior.dot(self.m.T)).T #1X3


    def error_model(self):
        self.zg = self.g - self.gAccel #1X3
        zm = self.mGyro - self.magReadings #1X3
        zp = np.concatenate((self.zg.reshape(-1,), zm.reshape(-1,)), axis=0) #1X6
        self.z = zp.reshape((1, 6))

    def magnetometer_correct(self):
        #Magnetometer Disturbance Error
        self.mError = (self.K[9:, :].dot(self.z.T)).T #3X1
        self.mError = self.mError.reshape((3,1))

        #Magnetic Jamming Detection
        if (np.sum(self.mError**2)) > 4*(self.ExpectedMageticFieldStrength**2):
            self.tf = True
        else :
            self.tf = False

    def kalman_equation(self):
        #Observation Model
        k = self.DecimationFactor/self.SampleRate
        self.H[0][1] = self.g[0][2] #gz
        self.H[0][2] = -self.g[0][1] #-gy
        self.H[0][4] = -k*self.g[0][2]
        self.H[0][5] = k*self.g[0][1]
        self.H[1][0] = -self.g[0][2]
        self.H[1][2] = self.g[0][0] #gx
        self.H[1][3] = k*self.g[0][2]
        self.H[1][5] = -k*self.g[0][0]
        self.H[2][0] = self.g[0][1]
        self.H[2][1] = -self.g[0][0]
        self.H[2][3] = -k*self.g[0][1]
        self.H[2][4] = k*self.g[0][0]

        self.H[3][1] = self.mGyro[0][2] #mgyroz
        self.H[3][2] = -self.mGyro[0][1] #-mgyroy
        self.H[3][4] = -k*self.mGyro[0][2]
        self.H[3][5] = k*self.mGyro[0][1]
        self.H[4][0] = -self.mGyro[0][2]
        self.H[4][2] = self.mGyro[0][0] #mgyrox
        self.H[4][3] = k*self.mGyro[0][2]
        self.H[4][5] = -k*self.mGyro[0][0]
        self.H[5][0] = self.mGyro[0][1]
        self.H[5][1] = -self.mGyro[0][0]
        self.H[5][3] = -k*self.mGyro[0][1]
        self.H[5][4] = k*self.mGyro[0][0] ##Observation model need to derivation?


        #Innovation Covariance
        accel_noise = self.AccelerometerNoise + self.LinearAccelerationNoise + (k**2)*(self.GyroscopedriftNoise + self.GyroscopeNoise)
        mag_noise = self.MagnetometerNoise + self.MagneticDisturbanceNoise + (k**2)*(self.GyroscopedriftNoise + self.GyroscopeNoise)
        self.R[0][0] = accel_noise
        self.R[1][1] = accel_noise
        self.R[2][2] = accel_noise
        self.R[3][3] = mag_noise
        self.R[4][4] = mag_noise
        self.R[5][5] = mag_noise
        self.S = self.R + self.H.dot(self.Pp.dot(self.H.T))


        #Update Error Estimate Covariance
        self.P = self.Pp - self.K.dot(self.H.dot(self.Pp))


        #Predict Error Estimate Covariance
        self.Q[0][0] = self.P[0][0] + (k**2)*self.P[3][3] + self.GyroscopedriftNoise + self.GyroscopeNoise
        self.Q[0][3] = -k*(self.P[3][3] + self.GyroscopedriftNoise)
        self.Q[1][1] = self.P[1][1] + (k**2)*(self.P[4][4] + self.GyroscopedriftNoise + self.GyroscopeNoise)
        self.Q[1][4] = -k*(self.P[4][4] + self.GyroscopedriftNoise)
        self.Q[2][2] = self.P[2][2] + (k**2)*self.P[5][5] + self.GyroscopedriftNoise + self.GyroscopeNoise
        self.Q[2][5] = -k*(self.P[5][5] + self.GyroscopedriftNoise)
        self.Q[3][0] = -k*(self.P[3][3] + self.GyroscopedriftNoise)
        self.Q[3][3] = self.P[3][3] + self.GyroscopedriftNoise
        self.Q[4][1] = -k*(self.P[4][4] + self.GyroscopedriftNoise)
        self.Q[4][4] = self.P[4][4] + self.GyroscopedriftNoise
        self.Q[5][2] = -k*(self.P[5][5] + self.GyroscopedriftNoise)
        self.Q[5][5] = self.P[5][5] + self.GyroscopedriftNoise
        self.Q[6][6] = (self.LinearAccelerationDecayFactor**2)*self.P[6][6] + self.LinearAccelerationNoise
        self.Q[7][7] = (self.LinearAccelerationDecayFactor**2)*self.P[7][7] + self.LinearAccelerationNoise
        self.Q[8][8] = (self.LinearAccelerationDecayFactor**2)*self.P[8][8] + self.LinearAccelerationNoise
        self.Q[9][9] = (self.MagneticDisturbanceDecayFactor**2)*self.P[9][9] + self.MagneticDisturbanceNoise
        self.Q[10][10] = (self.MagneticDisturbanceDecayFactor**2)*self.P[10][10] + self.MagneticDisturbanceNoise
        self.Q[11][11] = (self.MagneticDisturbanceDecayFactor**2)*self.P[11][11] + self.MagneticDisturbanceNoise
        self.Pp = self.Q


        #Kalman Gain
        self.inverse_term = np.linalg.inv(self.S.T)
        self.K = self.Pp.dot(self.H.T.dot(self.inverse_term))


        #Update a Posteriori Error
        self.x = self.K.dot(self.z.T) #12X1

        if self.tf == True: #magnetic jamming is detected in the current iteration
            self.x = self.K[:9, :3].dot(self.zg.T) #9X1
            self.x = self.x.reshape((9,1))


    def correct(self):
        #Estimate Orientation
        # self.q = self.q_hat.dot(self.x[:3, ]) #q, q_hat : 4X1 / theta+ : 3X1  #output1

        #Estimate Linear Acceleration
        self.linAccelprior = (self.linAccelprior)*self.LinearAccelerationDecayFactor - self.x[3:6, ].T
        self.linAccelprior = self.linAccelprior.reshape((1,3))

        #Estimate Gyroscope Offset
        self.gyroOffset = self.gyroOffset - self.x[6:9, ].T
        self.gyroOffset = self.gyroOffset.reshape((1,3))

        self._pose.pose.position.x = 0
        self._pose.pose.position.y = 0
        self._pose.pose.position.z = 0
        self._pose.pose.orientation.x = self.q[0][0]
        self._pose.pose.orientation.y = self.q[0][1]
        self._pose.pose.orientation.z = self.q[0][2]
        self._pose.pose.orientation.w = self.q[0][3]

        self._pose.header.stamp = rospy.Time.now()
        self._pose.header.frame_id = '/base_link'

        print("orientation:", self.q)

    def compute_angular_velocity(self):
        self.angular_velocity = np.sum(self.gyroreadings) - self.gyroOffset
        print("angular_velocity:", self.angular_velocity) #output2

    def update_magnetic_vector(self):
        self.rPost[0][0] = 2*(self.q[0][0]**2 + self.q[0][1]**2) - 1
        self.rPost[0][1] = 2*(self.q[0][1]*self.q[0][2] - self.q[0][0]*self.q[0][3])
        self.rPost[0][2] = 2*(self.q[0][1]*self.q[0][3] + self.q[0][0]*self.q[0][2])
        self.rPost[1][0] = 2*(self.q[0][1]*self.q[0][2] + self.q[0][0]*self.q[0][3])
        self.rPost[1][1] = 2*(self.q[0][0]**2 + self.q[0][2]**2) - 1
        self.rPost[1][2] = 2*(self.q[0][2]*self.q[0][3] - self.q[0][0]*self.q[0][1])
        self.rPost[2][0] = 2*(self.q[0][1]*self.q[0][3] - self.q[0][0]*self.q[0][2])
        self.rPost[2][1] = 2*(self.q[0][2]*self.q[0][3] + self.q[0][0]*self.q[0][1])
        self.rPost[2][2] = 2*(self.q[0][0]**2 + self.q[0][3]**2) - 1

        if self.tf == True: #if magnetic jamming was not detected in the current iteration, m skips updating
            mErrorNED = (self.rPost.T.dot(self.mError.T)).T #1X3
            M = self.m - mErrorNED #1X3
            liclination = math.atan2(M[0][2], M[0][0])

            self.m[0][0] = self.ExpectedMageticFieldStrength * math.cos(liclination)
            self.m[0][1] = 0
            self.m[0][2] = self.ExpectedMageticFieldStrength * math.sin(liclination)

    def Publish(self) :
        ang_vel = Vector3()
        ang_vel.x = self.angular_velocity[0][0]
        ang_vel.y = self.angular_velocity[0][1]
        ang_vel.z = self.angular_velocity[0][2]

        self._pub.publish(self._pose)
        self._angular_velocity_pub.publish(ang_vel)

if __name__ == '__main__' :
    rospy.init_node("ahrs_filter")
    Algorithm()
    rospy.spin()