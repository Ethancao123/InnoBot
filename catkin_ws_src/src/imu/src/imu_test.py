#!/usr/bin/env python
######################################################
# Copyright (c) 2020 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU9250/MPU9265 board
# (MPU6050 - accel/gyro, AK8963 - mag) to verify its
# correct wiring to a Raspberry Pi and the functionality
# of the MPU9250_i2c.py library
#
# Code modified to publish IMU data to ROS
######################################################
#

import time
import rospy
from sensor_msgs.msg import Imu
import numpy


t0 = time.time()
start_bool = False # boolean for connection
while (time.time()-t0)<5: # wait for 5-sec to connect to IMU
    try:
        from mpu9250_i2c import *
        start_bool = True # True for forthcoming loop
        break 
    except:
        continue
#
#############################
# Strings for Units/Labs
#############################
#
imu_devs   = ["ACCELEROMETER","GYROSCOPE","MAGNETOMETER"]
imu_labels = ["x-dir","y-dir","z-dir"]
imu_units  = ["m/s^2","m/s^2","m/s^2","rad/s","rad/s","rad/s","uT","uT","uT"]

#Create Node
def talker():
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = Imu()

#
#############################
#Loop to Test and Broadcast IMU
#############################
#
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "imu_link"
        if start_bool==False: # make sure the IMU was started
           print("IMU not Started, Check Wiring") # check wiring if error
           break
        msg.header.stamp = rospy.get_rostime()
    
    # Reading IMU values
        try:
            ax,ay,az,gx,gy,gz = mpu6050_conv() # read and convert mpu6050 data
        except:
            continue 
        q0 = 0.0 #W
        q1 = 0.0 #X
        q2 = 0.0 #Y
        q3 = 0.0 #Z

        t = time.time()
        currenttime = 0
        previoustime = currenttime
        currenttime = 1000000 * t + t / 1000000
        dt = (currenttime - previoustime) / 1000000.0
        if (dt < (1/1300.0)) : 
            time.sleep((1/1300.0 - dt) * 1000000)
        t = time.time()
        currenttime = 1000000 * t + t / 1000000
        dt = (currenttime - previoustime) / 1000000.0
        #print "Delta time: d = %f" % dt
        #Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)) :
            #Normalise accelerometer measurement
            recipNorm = (ax * ax + ay * ay + az * az)**-.5
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm
            #Estimated direction of gravity and vector perpendicular to magnetic flux
            halfvx = q1 * q3 - q0 * q2
            halfvy = q0 * q1 + q2 * q3
            halfvz = q0 * q0 - 0.5 + q3 * q3
            #Error is sum of cross product between estimated and measured direction of gravity
            halfex = (ay * halfvz - az * halfvy)
            halfey = (az * halfvx - ax * halfvz)
            halfez = (ax * halfvy - ay * halfvx)

        #Fill message
        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        msg.orientation.w = 0
        msg.orientation_covariance[0] = -1
        msg.orientation_covariance[0] = 0
        msg.orientation_covariance[0] = 0        

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = gx * gx
        msg.angular_velocity_covariance[4] = gy * gy
        msg.angular_velocity_covariance[8] = gz * gz
        
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance[0] = ax * ax
        msg.linear_acceleration_covariance[4] = ay * ay
        msg.linear_acceleration_covariance[8] = az * az
        
        pub.publish(msg)

        rate.sleep() # wait between prints

try:
    talker()
except rospy.ROSInterruptException:
    pass
    
