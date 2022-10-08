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
##################  GENERIC FUNCTIONS FOR BASIC MOVEMENT ################
class Drone: 
    def __init__(self, vehicle, input_matrix):
        global alt_reached, marker_heights, movementCoeff, last_seen
        #input_matrix = [ids_to_find,marker_sizes,marker_heights,takeoff_height]
        
        self.land_id_to_find = input_matrix[0]
        last_seen = np.zeros([len(input_matrix[0]),3])
        self.vehicle = vehicle
        self.takeoff_height = input_matrix[3]
        print(self.takeoff_height)
        marker_heights = input_matrix[2]
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
        self.horizontal_fov = 62.2 * (math.pi / 180)  # convert to radians 
        self.vertical_fov = 48.8 * (math.pi / 180)  # convert to radians
        self.dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]  # from the rostopic camera info
        self.camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
        self.np_camera_matrix = np.array(self.camera_matrix)
        self.np_dist_coeff = np.array(self.dist_coeff)

        self.found_count = [0]*len(input_matrix[0])
        self.notfound_count = [0]*len(input_matrix[0])  # keep track of aruco markers
        
        # keep track of time for update on precision landing
        self.time_last = 0.0
        self.time_to_wait = 0.1
        
        # image info
        self.newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)
        self.marker_size = input_matrix[1]

        # set PL parameters 
        # self.vehicle.parameters['PLND_ENABLED']=1
        # self.vehicle.parameters['PLND_TYPE']=1
        # # self.vehicle.parameters['PLND_TYPE']=3
        # self.vehicle.parameters['PLND_EST_TYPE']=0
        # self.vehicle.parameters['LAND_SPEED']= 20 # cm/s 

        # # set the rangefinder parameters 
        # self.vehicle.parameters["RNGFND1_TYPE"] = 1
        # self.vehicle.parameters["RNGFND1_MIN_CM"] = 0 
        # self.vehicle.parameters["RNGFND1_MAX_CM"] = 800 
        # self.vehicle.parameters["RNGFND1_PIN"] = 0  
        # self.vehicle.parameters["RNGFND1_SCALING"] = 12.12 
        
        
        # camera ros sub 
        rospy.init_node('drone_node', anonymous=False)
        self.vid_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)
        self.vid_sub_land = rospy.Subscriber('/camera/color/image_raw', Image, self.msg_receiver)

    # def connectMyCopter(self):
    #     # parser = argparse.ArgumentParser(description="commands")
    #     # parser.add_argument('--connect')
    #     # args = parser.parse_args()

    #     # connection_string = args.connect 

    #     # if not connection_string:
    #     #     import dronekit_sitl 
    #     #     sitl = dronekit_sitl.start_default()
    #     #     connection_string = sitl.connection_string() 
            
    #     # self.vehicle = connect(connection_string, wait_ready=True)
    #     vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

    #     return vehicle 
        
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
        self.vehicle.simple_takeoff(self.takeoff_height)  # meters 

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
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
            0b0000111111000111,  # .. BITMASK -> consider only the velocities 
            0, 0, 0,  # position
            vx, vy, vz,  # velocity 
            0, 0, 0,  # acceleration
            0, 0)
        print('moving foward')
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush() 


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
        self.vehicle.flush()

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
        self.vehicle.flush() 


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
        self.vehicle.flush() 

    def send_land_message(self, x, y): 
        # for precision landing
        msg = self.vehicle.message_factory.landing_target_encode(
                0,
                0,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                x,
                y,
                0,0,0
            )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush() 

    def land(self):
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
        self.vehicle.mode = VehicleMode('LAND')

        while self.vehicle.mode != "LAND":
            time.sleep(0.5)


    # #### PRECISION LANDING ######
    def msg_receiver(self, message):
        global notfound_count, found_count, time_last, time_to_wait, id_to_find, \
            alt_reached, movementCoeff, marker_heights

        #print('running land cmd')
        #self.vehicle.mode = VehicleMode('LAND')

        if time.time() - self.time_last > self.time_to_wait and alt_reached:
            np_data = rnp.numpify(message)  # deserialize image data into array 
            gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

            idstemp = []
            (corners, idstemp, rejected) = aruco.detectMarkers(image=gray_img, dictionary=self.aruco_dict, parameters=self.parameters)

            
            
            try:
                if idstemp is not None:
                    ids=list()
                    for x in idstemp:
                        ids.append(x[0])
                    #print(ids)
                    if len(marker_heights)>1:
                        land_id_to_find = 0
                        print(self.vehicle.location.global_relative_frame.alt)
                        for i in range(len(self.land_id_to_find)-1,-1,-1):
                            if self.vehicle.location.global_relative_frame.alt<=marker_heights[i]:
                                land_id_to_find = self.land_id_to_find[i]
                                marker_size = self.marker_size[i]
                                print('working----',land_id_to_find)
                                break
                                
                        if not land_id_to_find:
                            land_id_to_find = self.land_id_to_find[0]
                            marker_size = self.marker_size[0]
                            deltay = marker_heights[0] - self.vehicle.location.global_relative_frame.alt 
                            print('Going Down '+str(deltay))
                            self.goto_position_target_local_ned(0,0,deltay)
                    else:
                        land_id_to_find = self.land_id_to_find[0]
                    if land_id_to_find in ids:
                        k = np.where(np.array(ids)==land_id_to_find)[0][0]
                        #corners = cornerstemp[0][k]
                        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=self.np_camera_matrix,
                            distCoeffs=self.np_dist_coeff)
                        (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])  # rotation vctr and translation vctr
                        x = '{:.2f}'.format(tvec[0])  # x error 
                        y = '{:.2f}'.format(tvec[1])
                        z = '{:.2f}'.format(tvec[2])

                        y_sum = 0
                        x_sum = 0 
                        #print(ids[k])
                        #print(corners[k])
                        x_sum = corners[k][0][0][0] + corners[k][0][1][0] + corners[k][0][2][0] + corners[k][0][3][0]
                        y_sum = corners[k][0][0][1] + corners[k][0][1][1] + corners[k][0][2][1] + corners[k][0][3][1]
                        
                        if len(ids) > 1:
                            othermarks=[item for item in self.land_id_to_find if item not in [land_id_to_find]]
                            k2 = np.where(np.array(ids)==item)[0][0]
                        x_avg = x_sum /4 
                        y_avg = y_sum /4 

                        # two values required for the precision landing
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

                        marker_position = 'Marker Position: x='+str(x_avg)+' y='+str(y_avg)+' z='+z+''

                        aruco.drawDetectedMarkers(np_data, corners)
                        aruco.drawAxis(np_data, self.np_camera_matrix, self.np_dist_coeff, rvec, tvec, 10)  
                        cv2.putText(np_data, marker_position, (10,50), 0, 0.7, (255, 0,0), thickness=2)
                        print(marker_position)
                        print('FOUND COUNT:' + str(self.found_count) + " NOT FOUND COUNT:" + str(self.notfound_count))
                        self.found_count[i] += 1 
                    else: 
                        self.notfound_count[i] +=1 
                        print("Target Not Found")
                else: 
                    self.notfound_count[i] +=1 
            except Exception as e: 
                print("Target not found")
                print(e)
                self.notfound_count[i] += 1

            new_msg = rnp.msgify(Image, np_data, encoding="rgb8")
            self.newimg_pub.publish(new_msg)
            self.time_last = time.time()

        else:
            return None 
    
    
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
    
    
    def dict_msg_receiver(self, message):
        global IDsDict, time_last, time_to_wait, alt_reached, movementCoeff
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
                            IDsDict.loc['lastT'].iloc[i] = time.time()
                        
                    if land_id_to_find in ids:
                        IDsDict.loc['fcount'].iloc[i] += 1
                        landtag = True
                        
                    else:
                        IDsDict.loc['nfcount'].iloc[i] += 1
                        deltaT = time.time()-IDsDict[land_id_to_find].loc['lastT']
                        timelim = 2
                        if deltaT < timelim:
                            print("Target Not Found: Using last pos found",\
                                  str(deltaT),'seconds ago')
                            landtag = True
                        else: 
                            print("Target Not Found: Time limit elapsed ",\
                                  str(deltaT),'>',str(timelim))
                            landtag = False
                    
                    k2 = np.where(IDsDict.columns==land_id_to_find)[0][0]    
                    if landtag:
                        x_avg = IDsDict[land_id_to_find].loc['xloc']
                        y_avg = IDsDict[land_id_to_find].loc['yloc']
                        self.landScript(x_avg,y_avg)
        
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
    
    
    
    def subscriber(self):
        
        sub = rospy.Subscriber('/camera/color/image_raw', Image, self.dict_msg_receiver)
        rospy.spin()
        
def launcher(launchID, launchName):
    if launchID == 0:
        os.system('xdotool key "ctrl+shift+t";xdotool type "~/ardupilot/Tools/autotest/sim_vehicle.py --console -v ArduCopter";xdotool key Return')
        os.system('xdotool key "ctrl+shift+t";xdotool type "~/QGroundControl.AppImage";xdotool key Return')
    elif launchID == 1:
        os.system('xdotool key "ctrl+shift+t";xdotool type "~/ardupilot/Tools/autotest/sim_vehicle.py -f gazebo-iris --console -v ArduCopter";xdotool key Return')
        time.sleep(.6)
        os.system('xdotool key "ctrl+shift+t";xdotool type "roslaunch robotx_2022 '+launchName+'";xdotool key Return')
    elif launchID==2:
        os.system('xdotool key "ctrl+shift+t";xdotool type "~/ardupilot/Tools/autotest/sim_vehicle.py -f gazebo-iris --console -v ArduCopter";xdotool key Return')
        time.sleep(.6)
    os.system('xdotool key "alt+1"')
    time.sleep(.3)
    os.system('xdotool key "ctrl+shift+t";xdotool type "rqt";xdotool key Return')
    time.sleep(.3)