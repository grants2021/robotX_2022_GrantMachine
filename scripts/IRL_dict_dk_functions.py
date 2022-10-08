import time
import socket
import exceptions
import math
import argparse

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil

import rospy
import rosnumpy as rnp
#from sensor_msgs2.msg import Image
import cv2
import cv2.aruco as aruco 
import numpy as np
import pandas as pd

from imutils.video import WebcamVideoStream
import dict_dk_functions as dkf 
##################  GENERIC FUNCTIONS FOR BASIC MOVEMENT ################
class Drone: 
    def __init__(self, vehicle,IDsDictin,takeoff_height):
        global alt_reached, marker_heights, movementCoeff, last_seen, shutdowntag,\
            IDsDict, takeofftrueheight, landThreshold
        #input_matrix = [ids_to_find,marker_sizes,marker_heights,takeoff_height]
        IDsDict = IDsDictin
        landThreshold=0.4
        shutdowntag = False
        self.vehicle = vehicle
        self.takeoff_height = takeoff_height
        takeofftrueheight = self.vehicle.location.global_relative_frame.alt
        print(self.takeoff_height)
        movementCoeff = 1
        
        alt_reached = False
        print('running init')
        print(mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED)
        # aruco dict 
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters_create()

        # camera details
        self.horizontal_res = 640
        self.vertical_res = 480 
        self.cap = WebcamVideoStream(src=0).start()
        self.horizontal_fov = 62.2 * (math.pi / 180)  # convert to radians 
        self.vertical_fov = 48.8 * (math.pi / 180)  # convert to radians
        # self.dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]  # from the rostopic camera info
        # self.camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
        # self.np_camera_matrix = np.array(self.camera_matrix)
        # self.np_dist_coeff = np.array(self.dist_coeff)
        calib_path="/home/pi/video2calibration/calibrationFiles/"
        self.np_camera_matrix = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
        self.np_dist_coeff   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
        
        
        # keep track of time for update on precision landing
        self.time_last = 0.0
        self.time_to_wait = 0.1
        
        # image info
        self.newimg_pub = rospy.Publisher('/camera/color/image_new', self.cap.read(), queue_size=10)

        # set PL parameters 
        self.vehicle.parameters['PLND_ENABLED']=1
        self.vehicle.parameters['PLND_TYPE']=1
        self.vehicle.parameters['PLND_TYPE']=3
        self.vehicle.parameters['PLND_EST_TYPE']=0
        self.vehicle.parameters['LAND_SPEED']= 20 # cm/s 

        # set the rangefinder parameters 
        self.vehicle.parameters["RNGFND1_TYPE"] = 1
        self.vehicle.parameters["RNGFND1_MIN_CM"] = 0 
        self.vehicle.parameters["RNGFND1_MAX_CM"] = 1500 
        self.vehicle.parameters["RNGFND1_PIN"] = 0  
        self.vehicle.parameters["RNGFND1_SCALING"] = 10
        
        
        
        # camera ros sub 
        rospy.init_node('drone_node', anonymous=False)
        self.vid_pub = rospy.Publisher('/camera/color/image_new', self.cap.read(), queue_size=10)
        #self.vid_sub_land = rospy.Subscriber('/camera/color/image_raw', Image, self.msg_receiver)
    
    def arm(self):
        while self.vehicle.is_armable!=True:
            print("Waiting for vehicle to become armable.")
            time.sleep(2)
        print("Vehicle is now armable")
        
        self.vehicle.mode = "GUIDED"
        time.sleep()     
        while self.vehicle.mode!='GUIDED':
            print("Waiting for drone to enter GUIDED flight mode")
            time.sleep(2)
            self.vehicle.mode = "GUIDED"
            time.sleep(2)
            
        print("Vehicle now in GUIDED MODE. Have fun!!")

        self.vehicle.armed = True
        while self.vehicle.armed==False:
            print("Waiting for vehicle to become armed.")
            time.sleep(1)
        print("Look out! Virtual props are spinning!!")
        time.sleep(.5)

        return None

    def arm_and_takeoff(self):
        global alt_reached
        while self.vehicle.is_armable != True:
            print('Waiting for vehicle to become armable')
            time.sleep(1)

        print('vehicle is now armable')
        self.vehicle.mode = VehicleMode('GUIDED')

        while self.vehicle.mode != "GUIDED":
            print('Waiting for drone to enter guided flight mode')
            time.sleep(1)

        print('Vehicle now in guided mode')

        self.vehicle.armed = True 
        while self.vehicle.armed == False:
            print('Waiting for vehicle to become armed')
            time.sleep(1)

        print("props are spinning")
        self.vehicle.simple_takeoff(self.takeoff_height+takeofftrueheight)  # meters 

        while True:
            print("Current Altitude: %d"%self.vehicle.location.global_relative_frame.alt)
            print('Goal Alt: %d'%self.takeoff_height)
            if self.vehicle.location.global_relative_frame.alt>=0.9*self.takeoff_height:
                print("target altitude reached")
                alt_reached = True
                break 
            time.sleep(1)

        return None 

    def get_distance_meters(targetLocation, currentLocation):
        dLat = targetLocation.lat - currentLocation.lat
        dLon = targetLocation.lon - currentLocation.lon

        return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5 

    def goto(self, targetLocation):
        # input is a LocationGlobalRelative variable
        distanceToTargetLocation = self.get_distance_meters(targetLocation, self.vehicle.location.global_relative_frame)

        self.vehicle.simple_goto(targetLocation)

        while self.vehicle.mode.name == "GUIDED":
            currentDistance = self.get_distance_meters(targetLocation, self.vehicle.location.global_relative_frame)
            if currentDistance < distanceToTargetLocation*0.01:
                print("Reached target waypoint")
                time.sleep(2)
                break 
            time.sleep(1)

        return None

    def goto_position_target_local_ned(self,north, east, down):
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
        location in the North, East, Down frame.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down,
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    # send a velocity command with a +x being the heading of the drone 
    def send_local_ned_velocity(self, vx, vy, vz):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
            0b0000111111000111,  # .. BITMASK -> consider only the velocities 
            0, 0, 0,  # position
            vx, vy, vz,  # velocity 
            0, 0, 0,  # acceleration
            0, 0)
        #print('moving foward')
        self.vehicle.send_mavlink(msg)
        time.sleep(1)
        #self.vehicle.upload() 


    # send a velocity command with +x being the true north of earth 
    def send_global_ned_velocity(self, vx, vy, vz):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0,0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame  
            0b0000111111000111,  # .. BITMASK -> consider only the velocities 
            0, 0, 0,  # position
            vx, vy, vz,  # velocity 
            0, 0, 0,  # acceleration
            0, 0)
        self.vehicle.send_mavlink(msg)
        #self.vehicle.flush()

    def condition_yaw(self, degrees, relative):
        if relative:
            is_relative = 1  # yaw relative to direction of travel 

        else:
            is_relative = 0  # yaw is absolute angle 

        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component 
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command 
            0,  # confirmation 
            degrees,  # # param 1, yaw in degrees 
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw 
            is_relative,  # param 4, relative offset 1, absolute angle 0 
            0, 0, 0)  # param 5-7 not used 

        # send command to vehicle 
        self.vehicle.send_mavlink(msg)
        #self.vehicle.flush() 


    # fly to waypoint. we use ti as a dummy function to setup the condition_yaw function 
    def dummy_yaw_initializer(self):
        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon  
        alt = self.vehicle.location.global_relative_frame.alt

        aLocation=LocationGlobalRelative(lat, lon, alt)
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0,0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame  
            0b0000111111000111,  # .. BITMASK -> consider only the velocities 
            aLocation.lat*1e7,  # lat int - x position in wgs84 frame in 1e7 meters 
            aLocation.lon*1e7,  # lon int - y position in wgs84 frame in 1e7 meters  
            aLocation.alt,  # alt - altitude in meters 
            0, 0, 0,  # velocity 
            0, 0, 0,  # acceleration
            0, 0)

        # send command to vehicle 
        self.vehicle.send_mavlink(msg)
        #self.vehicle.flush() 

    def send_land_message(self,x,y):
        msg = self.vehicle.message_factory.landing_target_encode(
            0,
            0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            x,
            y,
            0,
            0,
            0,)
        self.vehicle.send_mavlink(msg)
        print(msg.__str__())
        self.vehicle.flush()
        
    
    def velLand(self,y_ang,x_ang,zreal,zoffset):
        x_ang = -x_ang
        vland = self.vehicle.parameters['LAND_SPEED']/100
        tland = zoffset/vland
        dx = math.atan(x_ang)*zreal
        dy = math.atan(y_ang)*zreal
        vx = (dx)/tland
        vy = (dy)/tland
        print('Marker at x:',dx,' y:',dy,' z:',zoffset)
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,0,
            mavutil.mavlink.MAV_FRAME_BODY_NED, 
            #0b0000111100011111,  # .. BITMASK -> consider only the vz ax xy 
            #0b0000111111000111,  # .. BITMASK -> consider only the vx vy vz
            0b0000111111000111,  # .. BITMASK -> consider only the dx dy vz
            0, 0, 0,  # position
            vx, vy, vland,  # velocity 
            0, 0, 0,  # acceleration
            0, 0)
        print(msg.__str__())
        self.vehicle.send_mavlink(msg)
        
    
    def horzAngleCalc(self,x):
        global momentCoeff
        return movementCoeff*(x - self.horizontal_res*0.5)*(self.horizontal_fov/self.horizontal_res)
    
    def vertAngleCalc(self,y):
        global momentCoeff
        return movementCoeff*(y - self.vertical_res*0.5)*(self.vertical_fov/self.vertical_res)
    
    def landScript(self,x_min,x_max,x_avg,y_min,y_max,y_avg,z,zoffset):
        global movementCoeff, shutdowntag, landThreshold
        
        x_size = self.horzAngleCalc(x_max)-self.horzAngleCalc(x_min)
        y_size = self.vertAngleCalc(y_max)-self.vertAngleCalc(y_min)
        x_ang = self.horzAngleCalc(x_avg)
        y_ang = self.vertAngleCalc(y_avg)
        
        if zoffset<1:
            zoffset=z
        if z <= (landThreshold):
            if self.vehicle.mode !="LAND":
                self.vehicle.mode = VehicleMode("LAND")
                while self.vehicle.mode != "LAND":
                    time.sleep(0.1)
                print("Vehicle is in LAND mode")
                shutdowntag = True
                #self.send_land_message(x_ang, y_ang)
        else:
            #self.send_land_message(x_ang, y_ang)
            self.velLand(x_ang,y_ang,z,zoffset)
        
        #print('Sent Land Message at x_ang=%0.5f'%x_ang+' y_ang=%0.5f'%y_ang)
    
    def rerouteScript(self,x_min,x_max,x_avg,y_min,y_max,y_avg,z,desiredZ):
        global movementCoeff, shutdowntag, landThreshold
        
        x_size = self.horzAngleCalc(x_max)-self.horzAngleCalc(x_min)
        y_size = self.vertAngleCalc(y_max)-self.vertAngleCalc(y_min)
        x_ang = self.horzAngleCalc(x_avg)
        y_ang = self.vertAngleCalc(y_avg)
        
        dx = -math.atan(y_ang)*z
        dy = -math.atan(x_ang)*z
        
        print('Marker at x:',dx,' y:',dy,' z:',z)
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,0,
            mavutil.mavlink.MAV_FRAME_BODY_NED, 
            #0b0000111100011111,  # .. BITMASK -> consider only the vz ax xy 
            #0b0000111111000111,  # .. BITMASK -> consider only the vx vy vz
            0b0000111111111000,  # .. BITMASK -> consider only the dx dy vz
            dx, dy, desiredZ,  # position
            0, 0, 0,  # velocity 
            0, 0, 0,  # acceleration
            0, 0)
        print(msg.__str__())
        self.vehicle.send_mavlink(msg)
        
        
    
    def msg_receiver(self, message):
        global IDsDict, alt_reached, movementCoeff, shutdowntag, landThreshold
        if shutdowntag:
            #print('Mission Completed')
            #shutdowntag = True
            rospy.signal_shutdown('Mission Completed')
            
        elif time.time() - self.time_last > self.time_to_wait and alt_reached:
            np_data = rnp.numpify(message)  # deserialize image data into array 
            gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
        
            idstemp = []
            (corners, idstemp, rejected) = aruco.detectMarkers(image=gray_img, dictionary=self.aruco_dict, parameters=self.parameters)
            
            land_id_to_find = 0
            for i in range(0,len(IDsDict.columns)):
                if self.vehicle.location.global_relative_frame.alt<=IDsDict.loc['height'].iloc[i]:
                    land_id_to_find = IDsDict.columns[i]
                    marker_size = IDsDict.loc['size'].iloc[i]
                    #print('working----',land_id_to_find)
                    if i!=0:
                        offset = self.vehicle.location.global_relative_frame.alt-IDsDict.loc['height'].iloc[i-1]
                    else:
                        offset = self.vehicle.location.global_relative_frame.alt-landThreshold
                    break
                        
            if not land_id_to_find and idstemp is not None:
                land_id_to_find = IDsDict.columns[-1]
                marker_size = IDsDict.loc['size'].iloc[-1]
                offset = self.vehicle.location.global_relative_frame.alt - IDsDict.loc['height'].iloc[-2] 
                #print('Going Down '+str(offset)+' to '+str(IDsDict.loc['height'].iloc[-1]))
                #self.goto_position_target_local_ned(0,0,deltay)
            
            if not land_id_to_find and idstemp is None:
                land_id_to_find = IDsDict.columns[-1]
                marker_size = IDsDict.loc['size'].iloc[-1]
                deltaz = IDsDict.loc['height'].iloc[-1] - self.vehicle.location.global_relative_frame.alt 
                print('Going Down '+str(deltaz)+' to '+str(IDsDict.loc['height'].iloc[-1]))
                self.goto_position_target_local_ned(0,0,deltaz)
            
            try:
                if idstemp is not None:
                    ids=list()
                    for X in idstemp:
                        ids.append(X[0])

                    #print(self.vehicle.location.global_relative_frame.alt)
                    rvec = [list()]*2
                    tvec = [list()]*2
                    for i in range(0,len(IDsDict.columns)):
                        if IDsDict.columns[i] in ids:
                            k = np.where(np.array(ids)==IDsDict.columns[i])[0][0]
                            #corners = cornerstemp[0][k]
                            
                            ret = aruco.estimatePoseSingleMarkers(corners[k], IDsDict.loc['size'].iloc[i], \
                                cameraMatrix=self.np_camera_matrix,distCoeffs=self.np_dist_coeff)
                            (rvec[i], tvec[i]) = (ret[0][0,0,:], ret[1][0,0,:])  # rotation vctr and translation vctr                                 
                            x = '{:.2f}'.format(tvec[i][0])  # x error 
                            y = '{:.2f}'.format(tvec[i][1])
                            z = '{:.2f}'.format(tvec[i][2])
            
                            
                            #print(ids[k])
                            #print(corners[k])
                            xcorns = [corners[k][0][0][0],corners[k][0][1][0],corners[k][0][2][0],corners[k][0][3][0]]
                            ycorns = [corners[k][0][0][1],corners[k][0][1][1],corners[k][0][2][1],corners[k][0][3][1]]
                            
                            
                            IDsDict.loc['xmin'].iloc[i] = min(xcorns)
                            IDsDict.loc['xmax'].iloc[i] = max(xcorns)
                            IDsDict.loc['xloc'].iloc[i] = sum(xcorns)/4 
                            IDsDict.loc['ymin'].iloc[i] = min(ycorns)
                            IDsDict.loc['ymax'].iloc[i] = max(ycorns)
                            IDsDict.loc['yloc'].iloc[i] = sum(ycorns)/4
                            IDsDict.loc['lastT'].iloc[i] = time.time()
                        
                    if land_id_to_find in ids:
                        IDsDict[land_id_to_find].loc['fcount'] += 1
                        id_to_find = land_id_to_find
                        reroutetag = False
                        landtag = True
                        
                    else:
                        IDsDict[land_id_to_find].loc['nfcount'] += 1
                        deltaT = time.time()-IDsDict[land_id_to_find].loc['lastT']
                        timelim = 2
                        if deltaT < timelim:
                            print("Target Not Found: Using last pos found "+\
                                  '%0.5f seconds ago'%deltaT)
                            id_to_find = land_id_to_find
                            reroutetag = False
                            landtag = True
                        else: 
                            id_to_find = ids[0]
                            reroutetag = True
                            landtag = False
                            print("Target Not Found: Heading to ID ",id_to_find)
                    
                    
                    k2 = np.where(IDsDict.columns==id_to_find)[0][0]
                    x_min = IDsDict[id_to_find].loc['xmin']
                    x_max = IDsDict[id_to_find].loc['xmax']
                    x_avg = IDsDict[id_to_find].loc['xloc']
                    y_min = IDsDict[id_to_find].loc['ymin']
                    y_max = IDsDict[id_to_find].loc['ymax']
                    y_avg = IDsDict[id_to_find].loc['yloc']
                    z = self.vehicle.location.global_relative_frame.alt
                    marker_position = 'Marker Position: x='+str(x_avg)+' y='+str(y_avg)+' z='+str(z)+''
                    
                    if landtag:
                        print('Landing @ ID ',id_to_find)
                        self.landScript(x_min,x_max,x_avg,y_min,y_max,y_avg,z,offset)
                        time.sleep(1)
                    
                    if reroutetag:
                        
                        desiredZ = IDsDict[land_id_to_find].loc['height']
                        self.rerouteScript(x_min,x_max,x_avg,y_min,y_max,y_avg,z,-z)
                        time.sleep(5)
                        
            
                    aruco.drawDetectedMarkers(np_data, corners)
                    aruco.drawAxis(np_data, self.np_camera_matrix, self.np_dist_coeff, rvec[k2], tvec[k2], 10)  
                    cv2.putText(np_data, marker_position, (10,50), 0, 0.7, (255, 0,0), thickness=2)
                    print('FOUND COUNT:'+str(IDsDict.columns.values)+str(IDsDict.loc['fcount'].values)\
                          +" NOT FOUND COUNT:"+str(IDsDict.columns.values)+str(IDsDict.loc['nfcount'].values)+'\n\n')
                else: 
                    IDsDict[land_id_to_find].loc['nfcount'] += 1 
            except Exception as e: 
                print("Target not found")
                print(e)
                IDsDict[land_id_to_find].loc['nfcount'] += 1 
        
            new_msg = rnp.msgify(self.cap.read(), np_data, encoding="rgb8")
            self.newimg_pub.publish(new_msg)
            self.time_last = time.time()
        else:
            return None 
    
    
    def subscriber(self):
        global shutdowntag
        while not rospy.is_shutdown():
            sub = rospy.Subscriber('/camera/color/image_raw', self.cap.read(), self.msg_receiver)
        #rospy.spin()
    
    