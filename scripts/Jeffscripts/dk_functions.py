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
# import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, APIException
from pymavlink import mavutil	
from array import array 

##################  GENERIC FUNCTIONS FOR BASIC MOVEMENT ################


# def connectMyCopter(self):
# 	# parser = argparse.ArgumentParser(description="commands")
# 	# parser.add_argument('--connect')
# 	# args = parser.parse_args()

# 	# connection_string = args.connect 

# 	# if not connection_string:
# 	# 	import dronekit_sitl 
# 	# 	sitl = dronekit_sitl.start_default()
# 	# 	connection_string = sitl.connection_string() 
		
# 	# self.vehicle = connect(connection_string, wait_ready=True)
# 	vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# 	return vehicle 

def arm(vehicle):
	while vehicle.is_armable!=True:
		print("Waiting for vehicle to become armable.")
		time.sleep(1)
	print("Vehicle is now armable")
    
	vehicle.mode = VehicleMode("GUIDED")
            
	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Vehicle now in GUIDED MODE. Have fun!!")

	vehicle.armed = True
	while vehicle.armed==False:
		print("Waiting for vehicle to become armed.")
		time.sleep(1)
	print("Look out! Virtual props are spinning!!")
	time.sleep(.5)

	return None

def arm_and_takeoff(vehicle, takeoff_height):

	while vehicle.is_armable != True:
		print('Waiting for vehicle to become armable')
		time.sleep(1)

	print('vehicle is now armable')
	vehicle.mode = VehicleMode('GUIDED')

	while vehicle.mode != "GUIDED":
		print('Waiting for drone to enter guided flight mode')
		time.sleep(1)

	print('Vehicle now in guided mode')

	vehicle.armed = True 
	while vehicle.armed == False:
		print('Waiting for vehicle to become armed')
		time.sleep(1)

	print("props are spinning")

	vehicle.simple_takeoff(takeoff_height)  # meters 

	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt >=0.99*takeoff_height:
			break 
		time.sleep(1)

	print("target altitude reached")

	return None 


def return_to_launch(vehicle, home):
	vehicle.mode = "RTL"

	if vehicle.mode != "RTL":
		time.sleep(0.1)

	distanceToTargetLocation = get_distance_meters(home, vehicle.location.global_relative_frame)
	currentDistance = distanceToTargetLocation
	
	while currentDistance < distanceToTargetLocation* 0.01:
		time.sleep(0.1)
		currentDistance = get_distance_meters(home, vehicle.location.global_relative_frame)
	# when the drone is within close proximity to the home spot, switch to landin
	vehicle.mode = "LAND"

	while vehicle.mode != "LAND":
		time.sleep(0.1)
	return None 


def get_distance_meters(argetLocation, currentLocation):
	dLat = argetLocation.lat - currentLocation.lat
	dLon = argetLocation.lon - currentLocation.lon

	return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5 

def get_gps_from_distance(start, bearing, distance):
	# will return the gps coordinates for a point a given distance and bearing from start
	R = 6378.1
	lat1 = math.radians(start[0]) #Current lat point converted to radians
	lon1 = math.radians(start[1]) #Current long point converted to radians

	lat2 = math.asin( math.sin(lat1)*math.cos(distance/R) +
     math.cos(lat1)*math.sin(distance/R)*math.cos(bearing))

	lon2 = lon1 + math.atan2(math.sin(bearing)*math.sin(distance/R)*math.cos(lat1),
             math.cos(distance/R)-math.sin(lat1)*math.sin(lat2))

	lat2 = math.degrees(lat2)
	lon2 = math.degrees(lon2)

	return lat2, lon2

def goto(vehicle, targetLocation):
	# input is a LocationGlobalRelative variable
	# distanceToTargetLocation = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
	vehicle.simple_goto(targetLocation)

	while vehicle.mode.name == "GUIDED":
		currentDistance = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
		if currentDistance <= 3.0:
			# print("Reached target waypoint")
			time.sleep(2)
			break 
		# print(currentDistance)
		time.sleep(1)

	return None

def goto_position_target_local_ned(vehicle, north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


# send a velocity command with a +x being the heading of the drone 
def send_local_ned_velocity(vehicle, vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0,0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
		0b0000111111000111,  # .. BITMASK -> consider only the velocities 
		0, 0, 0,  # position
		vx, vy, vz,  # velocity 
		0, 0, 0,  # acceleration
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush() 


# send a velocity command with +x being the true north of earth 
def send_global_ned_velocity(vehicle, vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,  # time_boot_ms (not used)
		0,0,  # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame  
		0b0000111111000111,  # .. BITMASK -> consider only the velocities 
		0, 0, 0,  # position
		vx, vy, vz,  # velocity 
		0, 0, 0,  # acceleration
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def condition_yaw(vehicle, degrees, relative):
	if relative:
		is_relative = 1  # yaw relative to direction of travel 

	else:
		is_relative = 0  # yaw is absolute angle 

	# create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0,  # target system, target component 
		mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command 
		0,  # confirmation 
		degrees,  # # param 1, yaw in degrees 
		0,  # param 2, yaw speed deg/s
		1,  # param 3, direction -1 ccw, 1 cw 
		is_relative,  # param 4, relative offset 1, absolute angle 0 
		0, 0, 0)  # param 5-7 not used 

	# send command to vehicle 
	vehicle.send_mavlink(msg)
	vehicle.flush() 


# fly to waypoint. we use ti as a dummy function to setup the condition_yaw function 
def dummy_yaw_initializer(vehicle):
	lat = vehicle.location.global_relative_frame.lat
	lon = vehicle.location.global_relative_frame.lon  
	alt = vehicle.location.global_relative_frame.alt  

	aLocation=LocationGlobalRelative(lat, lon, alt)
	msg = vehicle.message_factory.set_position_target_global_int_encode(
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
	vehicle.send_mavlink(msg)
	vehicle.flush() 

def send_land_message(vehicle, x, y): 
	# for precision landing
	msg = vehicle.message_factory.landing_target_encode(
			0,
			0,
			mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
			x,
			y,
			0,0,0
		)
	vehicle.send_mavlink(msg)
	vehicle.flush() 

def land(vehicle):
	vehicle.parameters['PLND_ENABLED']=1
	vehicle.parameters['PLND_TYPE']=1
	vehicle.parameters['PLND_TYPE']=3
	vehicle.parameters['PLND_EST_TYPE']=0
	vehicle.parameters['LAND_SPEED']= 30 # cm/s 

	# set the rangefinder parameters 
	vehicle.parameters["RNGFND1_TYPE"] = 1
	vehicle.parameters["RNGFND1_MIN_CM"] = 0 
	vehicle.parameters["RNGFND1_MAX_CM"] = 1500 
	vehicle.parameters["RNGFND1_PIN"] = 0  
	vehicle.parameters["RNGFND1_SCALING"] = 12.12 
	vehicle.mode = VehicleMode('LAND')

	while vehicle.mode != "LAND":
		time.sleep(0.5)


# #### PRECISION LANDING ######
def pl_msg_receiver(message):
	global vehicle, notfound_count, found_count, time_last, time_to_wait, id_to_find


	vehicle.mode = VehicleMode('LAND')

	if time.time() - time_last > time_to_wait:
		np_data = rnp.numpify(message)  # deserialize image data into array 
		gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

		ids = ''
		(corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

		try:
			if ids is not None:
				if ids[0]==land_id_to_find:
					ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=np_camera_matrix,
						distCoeffs=np_dist_coeff)
					(rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])  # rotation vctr and translation vctr
					x = '{:.2f}'.format(tvec[0])  # x error 
					y = '{:.2f}'.format(tvec[1])
					z = '{:.2f}'.format(tvec[2])

					y_sum = 0
					x_sum = 0 

					x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
					y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]
					
					x_avg = x_sum /4 
					y_avg = y_sum /4 

					# two values required for the precision landing
					x_ang = (x_avg - horizontal_res*0.5)*horizontal_fov/horizontal_res
					y_ang = (y_avg - vertical_res*0.5)*vertical_fov/vertical_res

					if vehicle.mode !="LAND":
						vehicle.mode = VehicleMode("LAND")
						while vehicle.mode != "LAND":
							time.sleep(0.1)
						print("Vehicle is in LAND mode")
						send_land_message(x_ang, y_ang)
					else:
						send_land_message(x_ang, y_ang)

					marker_position = 'Marker Position: x='+x+' y='+y+' z='+z+''

					aruco.drawDetectedMarkers(np_data, corners)
					aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)  
					cv2.putText(np_data, marker_position, (10,50), 0, 0.7, (255, 0,0), thickness=2)
					print(marker_position)
					print('FOUND COUNT:' + str(found_count) + " NOT FOUND COUNT:" + str(notfound_count))
					found_count += 1 
				else: 
					notfound_count +=1 
					print("Target Not Found")
			else: 
				notfound_count +=1 
		except Exception as e: 
			print("Target not found")
			print (e)
			notfound_count += 1

		new_msg = rnp.msgify(Image, np_data, encoding="rgb8")
		newimg_pub.publish(new_msg)
		time_last = time.time()

	else:
		return None 

def precision_land(vehicle):
	rospy.init_node('drone_node', anonymous=False)
	sub = rospy.Subscriber('/camera/color/image_raw', Image, pl_msg_receiver)
	rospy.spin()