#!/usr/bin/env python
"""
this is main processing file , this includes'
subscribers to all topic
publishing of master sting
importing of sub files to use their functions
saving all the values globally (v imp!)
"""

#importing required libraries
import rospy
import math
import cv2
import tf
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge() #this object will make transformation between ros and opencv format(both ways)
import numpy as np
import time
import sys

#importing assistant files(larger the number , more the priority)
import gps_and_yaw_processing_0
import camera_processing_1
import pitch_roll_processing_2
import lidar_processing_3

#importing messages
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#master string publisher
pub = rospy.Publisher('master_string_topic', String, queue_size=15)

#setting up fixed variables used in tuning
goal_lat  = 28.752588
goal_lon  = 77.120383
min_dist_to_stop=10.0 #rover will stop within 10m radius of the goal coordinates
straight_allowance=4.0 #rover motion will be considered straight within -4 to -4 degrees
rotation_allowance=20.0 #rover will start turning if required rotation is not in -20 to +20 degree
min_dist_for_camera_processing=10.0 #arrow and ball detection will start after this
#note: code is optimized to add marker locations one ob one

#setting up global variables
#1) for gps
rover_lat = 0.0
rover_lon = 0.0

#2) for imu
yaw   = 0.0
pitch = 0.0
roll  = 0.0

#3) for ball
ball_area=0.0
ball_cooridinate=0.0
is_ball_detected=0.0

#for arrow
arrow_direction=0.0

#4) for lidar
temp=LaserScan()
lidar_ranges=temp.ranges

#others
trigger=0
intr=0
Bearing=0.0
master_string=String()
master_string="00000"
master_string_old="00000"
# index 0 : motion type
# index 1 : pwm
# index 2 : pwm
# index 3 : pwm
# index 4 : interrupt
distance_lidar = 2.0
minimum_distance=0.0
rover_rotating_angle=0.0
intrp =0 


def calculation_and_display_function():
    #globalizing every value found
    global master_string, master_string_old , minimum_distance , rover_rotating_angle , trigger, rover_lat,rover_lon,goal_lat,goal_lon
    global intr, min_dist_for_camera_processing , Bearing , is_ball_detected
    master_string_old=master_string
    minimum_distance=gps_and_yaw_processing_0.calc_min_distance(rover_lat, rover_lon, goal_lat, goal_lon)
    rover_rotating_angle, Bearing =gps_and_yaw_processing_0.calc_rover_rotating_angle(rover_lat, rover_lon, goal_lat, goal_lon,yaw)
    
    

    if(lidar_processing_3.check_if_obstacle_detected(lidar_ranges,distance_lidar) == True):#if lidar detects obstacle
        intr=3
        master_string=master_string[0:4]+str(intr)
        print("lidar sensed obstacle !!!!")
        fav_path = lidar_processing_3.check_favourable_path(lidar_ranges,distance_lidar)

    elif(pitch_roll_processing_2.check_if_extreme_pitch_or_roll_detected() == True):#wrong pitch+roll values sensed
        intr=2
        master_string=master_string[0:4]+str(intr)
        print("imu sensed extreme roll and pitch values !!!!")#interrupt of string becomes 2
    elif((camera_processing_1.check_if_arrow_or_ball_detected()== 1) and (minimum_distance <= min_dist_for_camera_processing)):#camera detects ball or arrow
        intr=1
        master_string=master_string[0:4]+str(intr)
        print("camera detected something !!!!")#interrupt of string becomes 1
    else:
        intr=0
        master_string=master_string[0:4]+str(intr)
        print("gps and yaw based travel active .......")
    if(intrp == 1):
        if(lidar_processing_3.call_to_check(rover_rotating_angle,lidar_ranges,distance_lidar) == 1):
            intr=3
        else:
            intr=0
            intrp=0

    
    
    if(intr==3):#related to lidar_processing_3
        master_string,trigger = lidar_processing_3.master_string_generator(fav_path,2,5) #2 is the straight allowance and 5 is rotation allowance
    """if(intr==2):#related to pitch_roll_processing_2
        master_string=pitch_roll_processing_2.master_string_generator()
    if(intr==1):#related to camera_processing_1
        master_string=camera_processing_1.master_string_generator()"""
    if(intr==0):#related to gps_and_yaw_processing_0.py
        master_string,trigger=gps_and_yaw_processing_0.master_string_generator(minimum_distance, rover_rotating_angle,trigger,min_dist_to_stop,straight_allowance,rotation_allowance)

    #move below lines in listener later.
    print("")
    if(is_ball_detected==0):
        print("Ball detection status     : "+str(is_ball_detected))
    else:
        print("Ball detection status     : "+str(is_ball_detected)+"  Ball area : "+str(ball_area)+"  ball coordinate : "+str(ball_cooridinate))
    print("arrow detection sastus    : "+str(arrow_direction))
    print("obstacle detection status : 0.0")
    print("gps bearing               : "+str(Bearing))
    print("yaw from north            : "+str(yaw)+" degrees")
    print("goal  (lat , lon)         : ( "+str(goal_lat)+" , "+str(goal_lon)+" )")
    print("rover (lat , lon)         : ( "+str(rover_lat)+" , "+str(rover_lon)+" )")
    print("rover rotation required   : "+str(rover_rotating_angle)+" degrees")
    print("Distance to goal          : "+str(minimum_distance)+"  meters")
    print("master string             : "+str(master_string))
    
    if(master_string_old!=master_string):
        pub.publish("00500")
        print("!!Applying safety delay!!")
        print("")
        time.sleep(2)#comment this for manual travel mode
    else:
        pub.publish(master_string)
        print("")


#here callbacks are defined just to assign values in global variables
#all processing will be done in separate python file and then imported
def callback_gps(gps_data):
    global rover_lat,rover_lon,count
    rover_lat = gps_data.latitude  #present latitude
    rover_lon = gps_data.longitude #present longitude

def callback_lidar(laser_scan_data):
    global lidar_ranges
    lidar_ranges=laser_scan_data.ranges

def callback_ball(ball_data):
    global ball_area , ball_cooridinate , is_ball_detected
    ball_area=ball_data.x
    ball_cooridinate=ball_data.y
    is_ball_detected=ball_data.z

def callback_arrow(arrow_dir):
    global arrow_direction
    arrow_direction=arrow_dir.data

def callback_imu(imu_data):
    global yaw,pitch,roll
    #quaternion=[imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
    #yaw,pitch,roll = gps_and_yaw_processing_0.quaternion_to_ypr(quaternion) #it was earlier
    yaw=imu_data.x #it is new
    if(yaw>180):
        yaw=yaw-360
    calculation_and_display_function()
    
def listener():
    rospy.init_node('main_processing_node', anonymous=False)

    #subscribing all 4 types of sensors used in autonomous
    rospy.Subscriber("fix", NavSatFix , callback_gps)
    rospy.Subscriber("laser_scan_topic", LaserScan , callback_lidar)
    rospy.Subscriber("ball_data_topic", Point , callback_ball)
    rospy.Subscriber("arrow_direction_topic", Int16 , callback_arrow)
    rospy.Subscriber("imu_data", Point , callback_imu)#it is new

    #code at this line ------------ willwork only once ! wtf!!
    rospy.spin()

if __name__ == '__main__':
    print("Activating Autopilot Mode")
    listener()
