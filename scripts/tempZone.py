#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 22 12:03:27 2022

@author: grant
"""
import time 
# import exceptions
import math 
import argparse 
import rospy 
from sensor_msgs.msg import Image 
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
import os


ids_to_find = [72,129]
marker_sizes = [20,40] #cm
marker_heights = [4,7]
found_count = [0]*len(ids_to_find)
notfound_count = [0]*len(ids_to_find)
x_avg = [0]*len(ids_to_find)
y_avg = [0]*len(ids_to_find)
last_seen = [0]*len(ids_to_find)
velocity = 0
takeoff_height = 8 #m 

global IDsDict, time_last, time_to_wait, alt_reached, movementCoeff

def landScript(self,x_avg,y_avg):
    global movementCoeff
    x_ang = movementCoeff*(x_avg - self.horizontal_res*0.5)*self.horizontal_fov/self.horizontal_res
    y_ang = movementCoeff*(y_avg - self.vertical_res*0.5)*self.vertical_fov/self.vertical_res

    if self.vehicle.mode !="LAND":
        self.vehicle.mode = VehicleMode("LAND")
        while self.vehicle.mode != "LAND":
            time.sleep(0.1)
        print("Vehicle is in LAND mode")
        self.send_land_message(x_ang, y_ang)
    else:
        self.send_land_message(x_ang, y_ang)


IDsDict = pd.DataFrame([marker_sizes,marker_heights,found_count,notfound_count,\
                            x_avg,y_avg,last_seen],columns=ids_to_find,
                           index=['size','height','fcount','nfcount','xloc','yloc','lastT'])

alt_reached = True
time_last = 0
time_to_wait = 0.1

def dict_msg_receiver(self, message):
    if time.time() - time_last > time_to_wait and alt_reached:
        np_data = rnp.numpify(message)  # deserialize image data into array 
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
    
        idstemp = []
        (corners, idstemp, rejected) = aruco.detectMarkers(image=gray_img, dictionary=self.aruco_dict, parameters=self.parameters)
    
        try:
            if idstemp is not None:
                ids=list()
                for x in idstemp:
                    ids.append(x[0])
                
                land_id_to_find = 0
                print(self.vehicle.location.global_relative_frame.alt)
                rvec = [list()]*2
                tvec = [list()]*2
                for i in range(0,len(IDsDict.columns)):
                    if self.vehicle.location.global_relative_frame.alt<=IDsDict.loc['height'].iloc[i]:
                        land_id_to_find = IDsDict.columns[i]
                        marker_size = IDsDict.loc['size'].iloc[i]
                        print('working----',land_id_to_find)
                        break
                            
                    if not land_id_to_find:
                        land_id_to_find = IDsDict.columns[-1]
                        marker_size = IDsDict.loc['size'].iloc[-1]
                        deltay = marker_heights[0] - self.vehicle.location.global_relative_frame.alt 
                        print('Going Down '+str(deltay))
                        self.goto_position_target_local_ned(0,0,deltay)
                    
                    if IDsDict.columns[i] in ids:
                        k = np.where(np.array(ids)==IDsDict.columns[i])[0][0]
                        #corners = cornerstemp[0][k]
                        ret = aruco.estimatePoseSingleMarkers(corners, IDsDict.loc['size'].iloc[i], \
                            cameraMatrix=self.np_camera_matrix,distCoeffs=self.np_dist_coeff)
                        (rvec[i], tvec[i]) = (ret[0][0,0,:], ret[1][0,0,:])  # rotation vctr and translation vctr
                        x = '{:.2f}'.format(tvec[0])  # x error 
                        y = '{:.2f}'.format(tvec[1])
                        z = '{:.2f}'.format(tvec[2])
        
                        y_sum = 0
                        x_sum = 0 
                        #print(ids[k])
                        #print(corners[k])
                        x_sum = corners[k][0][0][0] + corners[k][0][1][0] + corners[k][0][2][0] + corners[k][0][3][0]
                        y_sum = corners[k][0][0][1] + corners[k][0][1][1] + corners[k][0][2][1] + corners[k][0][3][1]
                        
                        IDsDict.loc['xloc'].iloc[i] = x_sum /4 
                        IDsDict.loc['yloc'].iloc[i] = y_sum /4
                    
                if land_id_to_find in ids:
                    IDsDict.loc['fcount'].iloc[i] += 1 
                    
                else: 
                    IDsDict.loc['nfcount'].iloc[i] += 1 
                    print("Target Not Found: Using last pos found",\
                          str(time.time()-IDsDict[land_id_to_find].loc['lastT']),\
                              'seconds ago')
                
                
                k2 = np.where(IDsDict.columns==land_id_to_find)[0][0]    
                x_avg = IDsDict[land_id_to_find].loc['xloc']
                y_avg = IDsDict[land_id_to_find].loc['yloc']
                landScript(x_avg,y_avg)
    
                marker_position = 'Marker Position: x='+str(x_avg)+' y='+str(y_avg)+' z='+z+''
    
                aruco.drawDetectedMarkers(np_data, corners)
                aruco.drawAxis(np_data, self.np_camera_matrix, self.np_dist_coeff, rvec[k2], tvec[k2], 10)  
                cv2.putText(np_data, marker_position, (10,50), 0, 0.7, (255, 0,0), thickness=2)
                print(marker_position)
                print('FOUND COUNT:'+str(IDsDict.columns.values)+str(IDsDict.loc['fcount'].values)\
                      +" NOT FOUND COUNT:"+str(IDsDict.columns.values)+str(IDsDict.loc['fcount'].values))
            else: 
                IDsDict.loc['nfcount'].iloc[i] += 1 
        except Exception as e: 
            print("Target not found")
            print(e)
            IDsDict.loc['nfcount'].iloc[i] += 1 
    
        new_msg = rnp.msgify(Image, np_data, encoding="rgb8")
        self.newimg_pub.publish(new_msg)
        self.time_last = time.time()
        
