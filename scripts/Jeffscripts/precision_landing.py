#!/usr/bin/python3

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

##########################

velocity = 1.0
takeoff_height = 5 #m 
#########################

newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)
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
	global vehicle, notfound_count, found_count, time_last, time_to_wait, id_to_find


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

if __name__ == "__main__":
	try: 
		# vehicle connection
		vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

		####### add in the parameters ######
		vehicle.parameters['PLND_ENABLED']=1
		vehicle.parameters['PLND_TYPE']=1
		# vehicle.parameters['PLND_TYPE']=4
		vehicle.parameters['PLND_EST_TYPE']=0
		vehicle.parameters['LAND_SPEED']= 20 # cm/s 

		# set the rangefinder parameters 
		vehicle.parameters["RNGFND1_TYPE"] = 1 
		vehicle.parameters["RNGFND1_MIN_CM"] = 0 
		vehicle.parameters["RNGFND1_MAX_CM"] = 300 
		vehicle.parameters["RNGFND1_PIN"] = 0  
		vehicle.parameters["RNGFND1_SCALING"] = 12.12 
		print("Taking off...")
		dkf.arm_and_takeoff(vehicle, takeoff_height)
		time.sleep(1)
		print("Moving forward...")
		dkf.send_local_ned_velocity(vehicle, velocity, 0, 0)
		time.sleep(2)
		# send_local_ned_velocity(velocityx, velocityy, 0)
		# # send_local_ned_velocity(0, 0, 0)
		# dkf.land(vehicle)
		precision_land(vehicle)

		while vehicle.armed == True:
			print('Waiting for disarm...')
			time.sleep(0.1)

	except rospy.ROSInterruptException:
		pass 