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

##########################
# vehicle connection
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
vehicle.parameters['PLND_ENABLED']=1
vehicle.parameters['PLND_TYPE']=1
vehicle.parameters['PLND_TYPE']=4
vehicle.parameters['PLND_EST_TYPE']=0
vehicle.parameters['LAND_SPEED']= 30 # cm/s 

# set the rangefinder parameters 
vehicle.parameters["RNGFND1_TYPE"] = 1 
vehicle.parameters["RNGFND1_MIN_CM"] = 0 
vehicle.parameters["RNGFND1_MAX_CM"] = 1500 
vehicle.parameters["RNGFND1_PIN"] = 0  
vehicle.parameters["RNGFND1_SCALING"] = 12.12 

# velocityx = np.random.uniform(-0.5, 0.5)  # m/s 
# velocityy = np.random.uniform(-0.5, 0.5)  # m/s 
velocity = 0.3
takeoff_height = 10 #m 
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

### Functions
def connectMyCopter():
	parser = argparse.ArgumentParser(description="commands")
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect 

	if not connection_string:
		import dronekit_sitl 
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string() 
		
	vehicle = connect(connection_string, wait_ready=True)

	return vehicle 

def arm_and_takeoff(targetHeight):

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

	vehicle.simple_takeoff(targetHeight)  # meters 

	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt >=0.95*targetHeight:
			break 
		time.sleep(1)

	print("target altitude reached")

	return None 

def send_local_ned_velocity(vx, vy, vz):
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

def send_land_message(x,y):
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

def msg_receiver(message):
	global notfound_count, found_count, time_last, time_to_wait, id_to_find

	if time.time() - time_last > time_to_wait:
		np_data = rnp.numpify(message)  # deserialize image data into array 
		gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

		ids = ''
		(corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

		try:
			if ids is not None:
				if ids[0]==id_to_find:
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


def subscriber():
	rospy.init_node('drone_node', anonymous=False)
	sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
	rospy.spin()

if __name__ == "__main__":
	try: 
		print("Taking off...")
		arm_and_takeoff(takeoff_height)
		time.sleep(1)
		print("Moving forward...")
		send_local_ned_velocity(velocity, 0, 0)
		# send_local_ned_velocity(velocityx, velocityy, 0)
		# send_local_ned_velocity(0, 0, 0)
		time.sleep(5)
		subscriber()

	except rospy.ROSInterruptException:
		pass 