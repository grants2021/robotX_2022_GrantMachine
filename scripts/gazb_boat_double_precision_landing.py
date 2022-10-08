#!/usr/bin/python

import rospy
from sensor_msgs2.msg import Image
from std_msgs.msg import Bool
import cv2
import cv2.aruco as aruco 
import sys
import time
import math
import numpy as np
import pandas as pd
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, APIException
from pymavlink import mavutil    
from array import array 
import dict_dk_functions as dkf 
import os

sys.path.append(r'/home/grant/vrx_ws/src/adept-vrx/robotX_2022/nodes')
import spare_node as sn

gazeboOrQGC = 10 #0 for gazebo 1 for QGroundControl
#launchName = 'wamv_qr_empty.launch'
launchName = 'platformed_drone_wamv_qr_empty.launch'
##########################
if gazeboOrQGC <= 2:
   dkf.launcher(gazeboOrQGC, launchName)

connection_string = "udp:127.0.0.1:14550"
# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string))
vehicle = connect(connection_string, wait_ready=True)

ids_to_find = [72,129]
marker_sizes = [20.0,40.0] #cm
marker_heights = [4,7]
follow_time = 20
spawn_height = 1.33 #ground = 0 WAMV = 1.3 Platform = 1.33
takeoff_height = 8 #m 
wamv_landing = True



found_count = [0]*len(ids_to_find)
notfound_count = [0]*len(ids_to_find)
enotfound_count = [0]*len(ids_to_find)
x_avg = [0]*len(ids_to_find)
y_avg = [0]*len(ids_to_find)
last_seen = [0]*len(ids_to_find)
vx = -1
vy = -1
vz = 0

IDsDict = pd.DataFrame([marker_sizes,marker_heights,found_count,notfound_count,\
                            enotfound_count,x_avg,y_avg,\
                                last_seen],columns=ids_to_find,
                           index=['size','height','fcount','nfcount','enfcount',\
                                  'xloc','yloc','lastT'])
#%%
#########################
    
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

#taken_off_pub = rospy.Publisher('/drone/takeoff_tag',Bool,queue_size=1)    
#taken_off_pub.publish(False)
node = rospy.init_node('drone_node', anonymous=False)
takeoffnode = sn.startup(0.0)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

#%%

@vehicle.on_message('WARNING')
def lisetener(self, message):
    print(message,'------------------')

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
        # vehicle connection5
        # vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        # print("Taking off...")
        # dkf.arm_and_takeoff(vehicle, takeoff_height)
        # time.sleep(1)
        # print("Moving forward...")
        # dkf.send_local_ned_velocity(vehicle, velocity, 0, 0)
        # # send_local_ned_velocity(velocityx, velocityy, 0)
        # # send_local_ned_velocity(0, 0, 0)
        # dkf.land(vehicle) 
        
        drone = dkf.Drone(vehicle,IDsDict,takeoff_height,spawn_height)

        # drone.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        print('Connected...')
        # drone.takeoff_height = 10
        # drone.arm() 
        drone.arm_and_takeoff()
        takeoffnode.update(True)
        #drone.send_local_ned_velocity(vx,0,0)
        #time.sleep(5)
        #drone.send_local_ned_velocity(0,vy,0)
        #time.sleep(5)
        t_end = time.time()+follow_time
        while time.time() < t_end:
            print('Following WAMV')
            drone.wamv_vel_follower(0)
        #drone.get_wamv_info('wamv_local_vel')
        #drone.goto_position_target_local_ned(-50,10,-takeoff_height)
        #drone.return_for_land(1)
        alldone = drone.landing_subscriber(IDsDict,wamv_landing)
        while drone.vehicle.armed == True or not rospy.is_shutdown() or not alldone:
            try:    
                print('Waiting for disarm...')
                time.sleep(0.1)
            except rospy.ROSInterruptException or APIException or KeyboardInterrupt or SystemExit:
                break
            finally:
                break
    except rospy.ROSInterruptException or APIException or KeyboardInterrupt or SystemExit:
        print('Drone Core Loop Break')
        pass
    finally:
        sys.exit()
        #os.system('killall gazebo')
sys.exit()
