#!/usr/bin/python

import rospy
from sensor_msgs2.msg import Image
import cv2
import cv2.aruco as aruco 
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil    
from array import array 
import dk_functions as dkf 
import os

gazeboOrQGC = 0 #0 for gazebo 1 for QGroundControl
##########################

if gazeboOrQGC == 1:
    os.system('xdotool key "ctrl+shift+t";xdotool type "~/ardupilot/Tools/autotest/sim_vehicle.py --console -v ArduCopter";xdotool key Return')
    os.system('xdotool key "ctrl+shift+t";xdotool type "~/QGroundControl.AppImage";xdotool key Return')
elif gazeboOrQGC == 0:
    os.system('xdotool key "ctrl+shift+t";xdotool type "~/ardupilot/Tools/autotest/sim_vehicle.py -f gazebo-iris --console -v ArduCopter";xdotool key Return')
    time.sleep(.3)
    os.system('xdotool key "ctrl+shift+t";xdotool type "roslaunch gazebo_ros iris_world.launch verbose:=true";xdotool key Return')
os.system('xdotool key "alt+1"')
time.sleep(.3)
os.system('xdotool key "ctrl+shift+t";xdotool type "rqt";xdotool key Return')
time.sleep(.3)

connection_string = "127.0.0.1:14550"
# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string))
vehicle = connect(connection_string, wait_ready=True)

#vehicle = connect('127.0.0.1:11345', wait_ready=True)
#vehicle = connect('127.0.0.1:9002',wait_ready=True)
#vehicle.parameters['PLND_ENABLED']=1
#vehicle.parameters['PLND_TYPE']=1
#vehicle.parameters['PLND_EST_TYPE']=0
#vehicle.parameters['LAND_SPEED']=30

velocity = 0.5
takeoff_height = 4 #m 
#########################

newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)
id_to_find = 72 
marker_size = 20 # cm

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()


horizontal_res = 640 
vertical_res = 480 

horizontal_fov = 62.2 * (math.pi / 180)  # convert to radians 
vertical_fov = 48.8 * (math.pi / 180)  # convert to radians 

found_count = 0 
notfound_count = 0 

######### camera intrinsics #######

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]  # from the rostopic camera info
camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

time_last = 0 
time_to_wait = 0.1 

if __name__ == "__main__":
    try: 
        # vehicle connection
        # vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        # print("Taking off...")
        # dkf.arm_and_takeoff(vehicle, takeoff_height)
        # time.sleep(1)
        # print("Moving forward...")
        # dkf.send_local_ned_velocity(vehicle, velocity, 0, 0)
        # # send_local_ned_velocity(velocityx, velocityy, 0)
        # # send_local_ned_velocity(0, 0, 0)
        # dkf.land(vehicle)

        drone = dkf.Drone(vehicle)

        # drone.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        print('Connected...')
        # drone.takeoff_height = 10
        # drone.arm() 
        drone.arm_and_takeoff()
        drone.send_local_ned_velocity(velocity, 0,0)
        # drone.goto_position_target_local_ned(10,0,10)

        drone.subscriber()

        while drone.vehicle.armed == True:
            print('Waiting for disarm...')
            time.sleep(0.1)

    except rospy.ROSInterruptException:
        pass 
