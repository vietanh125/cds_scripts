#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from math import sin, asin,sqrt, atan2, pi
import time
gyro_x_cal = 0
gyro_y_cal = 0
gyro_z_cal = 0
angle_pitch = 0
angle_roll = 0
skip = 1001
angle_pitch_output = 0
angle_roll_output = 0

set_gyro_angles = False
max_value = -1000000
min_value = 1000000

def imu_cb(imu):

    # global skip, gyro_x_cal, gyro_y_cal, gyro_z_cal, angle_pitch, angle_roll, set_gyro_angles, angle_pitch_output, angle_roll_output
    # # get data from imu
    # gyro_x = imu.angular_velocity.x
    # gyro_y = imu.angular_velocity.y
    # gyro_z = imu.angular_velocity.z
    global min_value, max_value

    acc_x = imu.linear_acceleration.x
    acc_y = imu.linear_acceleration.y
    acc_z = imu.linear_acceleration.z

    pitch = (atan2(acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180) / pi

    max_value = max(pitch, max_value)
    min_value = min(pitch, min_value)
    print pitch, min_value, max_value

    # #setup
    # if skip > 1:
    #     gyro_x_cal += gyro_x
    #     gyro_y_cal += gyro_y
    #     gyro_z_cal += gyro_z
    #     skip -= 1
    #     return

    # elif skip == 1:
    #     gyro_x_cal /= 1000
    #     gyro_y_cal /= 1000
    #     gyro_z_cal /= 1000
    #     skip -= 1
    #     return
    
    # # substract offset values from raw gyro values
    # gyro_x -= gyro_x_cal
    # gyro_y -= gyro_y_cal
    # gyro_z -= gyro_z_cal

    # # gyro angle calculation: 0.000508905 = 1 / (30Hz x 65.5) 
    # angle_pitch += gyro_x * 0.000508905
    # angle_roll += gyro_y * 0.000508905

    # # 0.000008882 = 0.000508905 * pi / 180
    # angle_pitch += angle_roll * sin(gyro_z * 0.000008882)
    # angle_roll -= angle_pitch * sin(gyro_z * 0.000008882)

    # acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z))

    # angle_pitch_acc = asin(acc_y/acc_total_vector) * 57.296
    # angle_roll_acc = asin(float(acc_x/acc_total_vector)) * (-57.296)

    # angle_pitch_acc -= 0.0
    # angle_roll_acc -= 0.0

    # if set_gyro_angles:
    #     angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004
    #     angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004
    # else:
    #     angle_pitch = angle_pitch_acc
    #     angle_roll = angle_roll_acc
    #     set_gyro_angles = True

    # angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1
    # angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1
    # print 'angle = ', angle_pitch_output


imu_sub = rospy.Subscriber('/mpu_9250/imu', Imu, imu_cb, queue_size=1)
rospy.init_node('test')
rospy.spin()
