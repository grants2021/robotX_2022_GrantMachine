#!/usr/bin/python

import rospy 
from sensor_msgs.msg import Image 
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



###########################
# vehicle connection
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
time.sleep(5.0)
# print('cv2:' + cv2.__version__)

####### add in the parameters ######
vehicle.parameters['PLND_ENABLED']=1
vehicle.parameters['PLND_TYPE']=4
# vehicle.parameters['PLND_EST_TYPE']=0
vehicle.parameters['LAND_SPEED']= 30 # cm/s 
vehicle.parameters['SIM_SONAR_SCALE'] = 10 
vehicle.parameters['RNGFND1_TYPE'] = 1 
vehicle.parameters['RNGFND_SCALING'] = 10 
vehicle.parameters['RNGFND1_PIN'] = 0 
vehicle.parameters['RNGFND1_MAX_CM'] = 800
vehicle.parameters['RNGFND1_MIN_CM'] = 0 
vehicle.parameters['WPNAV_SPEED'] = 500.0  # cm/s

##########################

##########################

velocity = 8.0
takeoff_height = 5 #m 
#########################

newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)
# rospy.init_node('drone_node_search', anonymous=False)
land_id_to_find = 72 
marker_size = 20 # cm

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()


horizontal_res = 640 
vertical_res = 480 

horizontal_fov = 62.2 * (math.pi / 180)  # convert to radians 
vertical_fov = 48.8 * (math.pi / 180)  # convert to radians 

found_count = 0 
notfound_count = 0 


######### info on the markers to find #######
marker1_id = 18
marker2_id = 35 
markers_id = [marker1_id, marker2_id]
marker1_found = False 
marker2_found = False 

#############################################

######### camera intrinsics #######

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]  # from the rostopic camera info
camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

time_last = 0 
time_to_wait = 0.1 


############ precision landing scripts ############

def send_land_message(x, y): 
	global vehicle 
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


def pl_msg_receiver(message):
	global vehicle, notfound_count, found_count, time_last, time_to_wait, land_id_to_find


	# vehicle.mode = VehicleMode('LAND')

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
			print e
			notfound_count += 1

		new_msg = rnp.msgify(Image, np_data, encoding="rgb8")
		newimg_pub.publish(new_msg)
		time_last = time.time()

	else:
		return None 

def precision_land(vehicle):
	print('Landing...')
	rospy.init_node('drone_node', anonymous=False)
	sub = rospy.Subscriber('/camera/color/image_raw', Image, pl_msg_receiver)
	# rospy.spin()
	while vehicle.armed == True:
		time.sleep(1)
	return None 

def scan_targets(message):
	global vehicle, notfound_count, found_count, time_last, time_to_wait,marker1_found, marker2_found, marker1_id, marker2_id
	
	if time.time() - time_last > time_to_wait:
		np_data = rnp.numpify(message)  # deserialize image data into array 
		gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

		ids = ''
		(corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

		try:
			if ids is not None:
				for id in ids:
					if ids[0]==marker1_id or ids[0]==marker2_id:
						# print(ids[0])
						# print("FOUND ONE")

						if ids[0]==marker1_id:
							marker1_found = True

						if ids[0]==marker2_id:
							marker2_found = True

						# get the location of the marker
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

						marker_position = 'Marker Position: x='+x+' y='+y+' z='+z+''

						# draw the aruco marker 
						aruco.drawDetectedMarkers(np_data, corners)
						aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)  
						cv2.putText(np_data, marker_position, (10,50), 0, 0.7, (255, 0,0), thickness=2)
						# print(marker_position)

		except Exception as e: 
			print('Nothing Found...')
			return None

		new_msg = rnp.msgify(Image, np_data, encoding="rgb8")
		newimg_pub.publish(new_msg)
		time_last = time.time()

	else:
		return None   


def search(vehicle, bound_gps):
	"""A function to search the field for the targets"""
	global marker1_found, marker2_found
	rospy.init_node('drone_node', anonymous=False)
	sub = rospy.Subscriber('/camera/color/image_raw', Image, scan_targets)

	[SW_gps, NW_gps, NE_gps, SE_gps] = bound_gps

	# create an array of gps points for the drone to travel to based on boundary gps
	lon_points = np.linspace(SE_gps[1], SW_gps[1], num=4)
	lat_points = np.linspace(SW_gps[0], NW_gps[0], num=5)

	points = [[lat_points[0], lon_points[1]],
	[lat_points[0], lon_points[2]],		
	[lat_points[1], lon_points[2]],		
	[lat_points[1], lon_points[1]],
	[lat_points[2], lon_points[1]],
	[lat_points[2], lon_points[2]],
	[lat_points[3], lon_points[2]],
	[lat_points[3], lon_points[1]],
	[lat_points[4], lon_points[1]],
	[lat_points[4], lon_points[2]]]
	# execture the mission

	for point in points:
		dkf.goto(vehicle, LocationGlobalRelative(point[0], point[1], takeoff_height))
		time.sleep(1.0)

		if marker1_found==True and marker2_found == True:
			sub.unregister()
			break


	return None 


if __name__ == "__main__":
	try: 
 
		# takeoff 
		print("Taking off...")
		dkf.arm_and_takeoff(vehicle, takeoff_height)
		time.sleep(3.0)

		# record this location for the return 
		home_gps = vehicle.location.global_relative_frame
		lat_home = vehicle.location.global_relative_frame.lat 
		lon_home = vehicle.location.global_relative_frame.lon
		# print(home_gps.lat)
		wp_home = LocationGlobalRelative(lat_home, lon_home, takeoff_height)
		heading = vehicle.heading  # degrees  

		# get the location of the boundaries to use for waypoint generation 
		SE_bound = [-15, -5]
		SW_bound = [-15,5]
		NW_bound = [15,5]
		NE_bound = [15,-5]
		drone_home = [-16, -5]

		SE_bound_dist = SE_bound[0]-drone_home[0]
		SE_gps = dkf.get_gps_from_distance([lat_home, lon_home], 0.0, np.abs(SE_bound_dist) / 1000.0)
		SW_gps = dkf.get_gps_from_distance([SE_gps[0], SE_gps[1]],270.0*math.pi/180.0 , np.abs((SW_bound[1]-SE_bound[1]))/1000.0)
		NW_gps = dkf.get_gps_from_distance([SW_gps[0], SW_gps[1]], 0.0*math.pi/180.0, np.abs((NW_bound[0]-SW_bound[0]))/1000.0)
		NE_gps = dkf.get_gps_from_distance([NW_gps[0], NW_gps[1]], 90.0*math.pi/180.0, np.abs((NW_bound[1]-NE_bound[1]))/1000.0)

		bound_gps = [SW_gps, NW_gps, NE_gps, SE_gps]
		# print(home_gps)
		# for point in bound_gps:
		# 	# print(point)
		# 	dkf.goto(vehicle, LocationGlobalRelative(point[0], point[1], takeoff_height))
		# 	time.sleep(1.0)

		# create an array of gps points for the drone to travel to based on boundary gps
		# lon_points = np.linspace(SE_gps[1], SW_gps[1], num=4)
		# lat_points = np.linspace(SW_gps[0], NW_gps[0], num=5)
		# print('Lon:', lon_points)
		# print('Lat:', lat_points)

		# points = [[lat_points[0], lon_points[1]],
		# [lat_points[0], lon_points[2]],		
		# [lat_points[1], lon_points[2]],		
		# [lat_points[1], lon_points[1]],
		# [lat_points[2], lon_points[1]],
		# [lat_points[2], lon_points[2]],
		# [lat_points[3], lon_points[2]],
		# [lat_points[3], lon_points[1]],
		# [lat_points[4], lon_points[1]],
		# [lat_points[4], lon_points[2]]]
		# execture the mission
		search(vehicle, bound_gps)
		# for point in points:
		# 	print(point)
		# 	dkf.goto(vehicle, LocationGlobalRelative(point[0], point[1], takeoff_height))
		# 	time.sleep(1.0)



		# execute the mission 
		# side_movement = 0.005  # km
		# forward_movement = 0.005  # km
		# forward_distance = 20.0 
		# traveled_forward_distance = 0.0 
		# left_heading = (heading - 90)* math.pi /180.0  # radians 
		# right_heading = (heading + 90) * math.pi / 180.0 
		# print("Beginning Search...")
		# while traveled_forward_distance <= forward_distance:
		# 	# go forward and to the side 
		# 	print('moving left...')
		# 	current_point = vehicle.location.global_relative_frame
		# 	point2 = dkf.get_gps_from_distance([current_point.lat, current_point.lon], 270.0*math.pi/180.0, side_movement)
		# 	wp_2 = LocationGlobalRelative(point2[0], point2[1], takeoff_height)
		# 	dkf.goto(vehicle, wp_2)
		# 	time.sleep(1)

		# 	print('moving forward...')
		# 	current_point = vehicle.location.global_relative_frame
		# 	point2 = dkf.get_gps_from_distance([current_point.lat, current_point.lon], 0.0, forward_movement)
		# 	wp_2 = LocationGlobalRelative(point2[0], point2[1], takeoff_height)
		# 	dkf.goto(vehicle, wp_2)
		# 	time.sleep(1)

		# 	# go forward and to the other side 
		# 	print('moving right...')
		# 	current_point = vehicle.location.global_relative_frame
		# 	point2 = dkf.get_gps_from_distance([current_point.lat, current_point.lon], 90.0*math.pi/180.0, side_movement)
		# 	wp_2 = LocationGlobalRelative(point2[0], point2[1], takeoff_height)
		# 	dkf.goto(vehicle, wp_2)
		# 	time.sleep(1)

		# 	print('moving forward again...')
		# 	current_point = vehicle.location.global_relative_frame
		# 	point2 = dkf.get_gps_from_distance([current_point.lat, current_point.lon], 0.0, forward_movement)
		# 	wp_2 = LocationGlobalRelative(point2[0], point2[1], takeoff_height)
		# 	dkf.goto(vehicle, wp_2)
		# 	time.sleep(1)

		# 	traveled_forward_distance = traveled_forward_distance + 2*forward_movement * 1000.0
		#################################3


		# return to launch and land 
		print('returning to launch...')
		dkf.goto(vehicle, home_gps)
		# dkf.return_to_launch(vehicle, home)

		# land 
		# print('Landing...')
		precision_land(vehicle)

		# while vehicle.armed == True:
		# 	print('Waiting for disarm...')
		# 	time.sleep(0.1)

	except rospy.ROSInterruptException:
		pass 