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


connection_string = "udp:127.0.0.1:14550"
# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string))
vehicle = connect(connection_string, wait_ready=True)
spawn_height = 0 #ground = 0 WAMV = 1.3 Platform = 1.33
takeoff_height = 5 #m 
flight_height = takeoff_height

#%%
#####Task Landing Values#####
LP_ids = [0,1] #0 = empty 1 = full
LP_sizes = [200,200] #cm
LP_heights = [7,7]
wamv_landing = False

LP_found_count = [0]*len(LP_ids)
LP_notfound_count = [0]*len(LP_ids)
LP_enotfound_count = [0]*len(LP_ids)
LP_x_avg = [0]*len(LP_ids)
LP_y_avg = [0]*len(LP_ids)
LP_last_seen = [0]*len(LP_ids)

LP_Dict = pd.DataFrame([LP_sizes,LP_heights,LP_found_count,\
                        LP_notfound_count,LP_enotfound_count,LP_x_avg,LP_y_avg,\
                            LP_last_seen],columns=LP_ids,
                           index=['size','height','fcount','nfcount','enfcount',\
                                  'xloc','yloc','lastT'])


#%%
#####WAMV Landing values#####
ids_to_find = [72,129]
marker_sizes = [20.0,40.0] #cm
marker_heights = [4,7]
follow_time = 20
wamv_landing = True

found_count = [0]*len(ids_to_find)
notfound_count = [0]*len(ids_to_find)
enotfound_count = [0]*len(ids_to_find)
x_avg = [0]*len(ids_to_find)
y_avg = [0]*len(ids_to_find)
last_seen = [0]*len(ids_to_find)

IDsDict = pd.DataFrame([marker_sizes,marker_heights,found_count,notfound_count,\
                            enotfound_count,x_avg,y_avg,\
                                last_seen],columns=ids_to_find,
                           index=['size','height','fcount','nfcount','enfcount',\
                                  'xloc','yloc','lastT'])
#%%
    
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

#taken_off_pub = rospy.Publisher('/drone/takeoff_tag',Bool,queue_size=1)    
#taken_off_pub.publish(False)
node = rospy.init_node('drone_node', anonymous=False)
takeoffnode = sn.startup(0.0)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

#%%
'''
@vehicle.on_message('WARNING')
def lisetener(self, message):
    print(message,'------------------')
'''
#%%
######Camera intrinsics######
horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180)  # convert to radians 
vertical_fov = 48.8 * (math.pi / 180)  # convert to radians 

found_count = 0 
notfound_count = 0 



dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]  # from the rostopic camera info
camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

time_last = 0 
time_to_wait = 0.1 

#%%
#####Main Script#####

if __name__ == "__main__":
    try: 
        drone = dkf.Drone(vehicle,IDsDict,LP_Dict,takeoff_height,spawn_height)

        print('Connected...')
       
        drone.arm_and_takeoff()
        takeoffnode.update(True)
        
        #drone.goto_position_target_local_ned(10,5,-flight_height)
        #drone.send_local_ned_velocity(5,0,0)
        #drone.simple_look()
        '''
        alldone = drone.landing_subscriber(IDsDict,wamv_landing)
        while drone.vehicle.armed == True or not rospy.is_shutdown() or not alldone:
            try:    
                print('Waiting for disarm...')
                time.sleep(0.1)
            except rospy.ROSInterruptException or APIException or KeyboardInterrupt or SystemExit:
                break
            finally:
                break
        '''
    except rospy.ROSInterruptException or APIException or KeyboardInterrupt or SystemExit as e:
        print('Drone Core Loop Break')
        print(e)
        pass
    finally:
        pass
        #sys.exit()
        #os.system('killall gazebo')
#sys.exit()
