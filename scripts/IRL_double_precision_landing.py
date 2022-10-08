#!/usr/bin/python
import time
import socket
import exceptions
import math
import argparse

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil

import rospy
#from sensor_msgs2.msg import Image
import cv2
import cv2.aruco as aruco 
import numpy as np
import pandas as pd

from imutils.video import WebcamVideoStream
import dict_dk_functions as dkf 

def connectMyCopter():

	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect

	if not connection_string:
            connection_string='127.0.0.1:14550'

	vehicle = connect(connection_string,wait_ready=True, baud=921600)

	return vehicle

vehicle=connectMyCopter()

ids_to_find = [72,129]
marker_sizes = [20.0,40.0] #cm
marker_heights = [4,7]
found_count = [0]*len(ids_to_find)
notfound_count = [0]*len(ids_to_find)
enotfound_count = [0]*len(ids_to_find)
x_min = [0]*len(ids_to_find)
x_max = [0]*len(ids_to_find)
x_avg = [0]*len(ids_to_find)
y_min = [0]*len(ids_to_find)
y_max = [0]*len(ids_to_find)
y_avg = [0]*len(ids_to_find)
last_seen = [0]*len(ids_to_find)
vx = -1
vy = -1
vz = 0
takeoff_height = 8 #m 

IDsDict = pd.DataFrame([marker_sizes,marker_heights,found_count,notfound_count,\
                            enotfound_count,x_min,x_max,x_avg,y_min,y_max,y_avg,\
                                last_seen],columns=ids_to_find,
                           index=['size','height','fcount','nfcount','enfcount',\
                                  'xmin','xmax','xloc','ymin','ymax','yloc','lastT'])
#%%
#########################

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
        
        drone = dkf.Drone(vehicle,IDsDict,takeoff_height)

        # drone.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        print('Connected...')
        # drone.takeoff_height = 10
        # drone.arm() 
        drone.arm_and_takeoff()
        drone.send_local_ned_velocity(vx,0,0)
        time.sleep(5)
        drone.send_local_ned_velocity(0,vy,0)
        time.sleep(5)
        # drone.goto_position_target_local_ned(10,10,10)
        
        drone.subscriber()

        #while drone.vehicle.armed == True:
        #    print('Waiting for disarm...')
        #    time.sleep(0.1)

    except rospy.ROSInterruptException:
        pass 
