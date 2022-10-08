#!/usr/bin/python3

import rospy 
from sensor_msgs.msg import Image 
import cv2
import cv2.aruco as aruco 
import sys
import time
import math
import numpy as np
# import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil	
from array import array 
import dk_functions as dkf 
import os
import pytesseract
from pytesseract import Output
from imutils.object_detection import non_max_suppression
from cv_bridge import CvBridge
import imutils


###########################
# vehicle connection
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
time.sleep(5.0)
# print('cv2:' + cv2.__version__)

####### add in the parameters ######
# vehicle.parameters['PLND_ENABLED']=1
# vehicle.parameters['PLND_TYPE']=4
# # vehicle.parameters['PLND_EST_TYPE']=0
# vehicle.parameters['LAND_SPEED']= 30 # cm/s 
# vehicle.parameters['SIM_SONAR_SCALE'] = 10 
# vehicle.parameters['RNGFND1_TYPE'] = 1 
# vehicle.parameters['RNGFND_SCALING'] = 10 
# vehicle.parameters['RNGFND1_PIN'] = 0 
# vehicle.parameters['RNGFND1_MAX_CM'] = 300
# vehicle.parameters['RNGFND1_MIN_CM'] = 0 

vehicle.parameters['PLND_ENABLED']=1
vehicle.parameters['PLND_TYPE']=1
# vehicle.parameters['PLND_TYPE']=4
vehicle.parameters['PLND_EST_TYPE']=0
vehicle.parameters['LAND_SPEED']= 20 # cm/s 

# set the rangefinder parameters 
vehicle.parameters["RNGFND1_TYPE"] = 1 
vehicle.parameters["RNGFND1_MIN_CM"] = 0 
vehicle.parameters["RNGFND1_MAX_CM"] = 400 
vehicle.parameters["RNGFND1_PIN"] = 0  
vehicle.parameters["RNGFND1_SCALING"] = 12.12 
vehicle.parameters['WPNAV_SPEED'] = 500.0  # cm/s

##########################
##########################

velocity = 8.0
takeoff_height = 5.0 #m 
# vehicle.parameters["RNGFND1_MAX_CM"] = 0.8*takeoff_height
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
marker1_id = "N"
marker2_id = "R" 
markers_id = [marker1_id, marker2_id]
marker1_found = False 
marker2_found = False 
search_now = False  # only look for text while the drone is stationary at the waypoint

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


	vehicle.mode = VehicleMode('LAND')

	while vehicle.mode != 'LAND':
		vehicle.mode = VehicleMode('LAND')
		time.sleep(1.0)

	if time.time() - time_last > time_to_wait:
		# np_data = rnp.numpify(message)  # deserialize image data into array 

		# try the new way with the cv bridge
		bridge = CvBridge()
		np_data = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
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
			# print e
			notfound_count += 1

		# the old way using ros_numpy	
		# new_msg = rnp.msgify(Image, np_data, encoding="rgb8")

		# the new way using cvbridge
		new_msg = bridge.cv2_to_imgmsg(np_data, encoding="rgb8")
		newimg_pub.publish(new_msg)
		time_last = time.time()

	else:
		return None 


class ShapeDetector:
	def __init__(self):
		pass

	def detect(self,c):
		# initialize the shape name and approximate contour 
		shape = 'unidentified'
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04*peri, True)

		if len(approx) ==3:
			shape = "triangle"

		elif len(approx) == 4: 
			(x,y,w,h) = cv2.boundingRect(approx)
			ar = w/float(h)

			shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"

		elif len(approx) == 5: 
			shape = "pentagon"

		else: 
			shape = 'circle'

		return shape 


def pl_msg_receiver_landing_pad(message):
	global vehicle, notfound_count, found_count, time_last, time_to_wait, land_id_to_find


	vehicle.mode = VehicleMode('LAND')

	while vehicle.mode != 'LAND':
		vehicle.mode = VehicleMode('LAND')
		time.sleep(1.0)

	if time.time() - time_last > time_to_wait:
		# np_data = rnp.numpify(message)  # deserialize image data into array 

		# try the new way with the cv bridge
		bridge = CvBridge()
		np_data = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
		gray = get_grayscale(np_data)
		thresh = thresholding(gray)
		thresh = noise_removal(thresh)

		# find the contours in the thresholded image 
		cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts=imutils.grab_contours(cnts)

		# loop over the contours 
		sd = ShapeDetector()

		# get the vehicle altitude to determine if need to find square or circle 
		current_alt = vehicle.location.global_relative_frame.alt
		if current_alt >= 1.0:
			target_shape = "square"

		else: target_shape = "square"

		try:
			for c in cnts:
				# compute the center of the contour
				shape = sd.detect(c)

				if shape == target_shape:
					M=cv2.moments(c)
					cX = int(M['m10']/M['m00'])
					cY = int(M['m01']/M["m00"])

					# use the center point to guide the precision landing
					x_ang = (cX - horizontal_res*0.5)*horizontal_fov/horizontal_res
					y_ang = (cY- vertical_res*0.5)*vertical_fov/vertical_res

					# land the vehicle 
					if vehicle.mode !="LAND":
						vehicle.mode = VehicleMode("LAND")
						while vehicle.mode != "LAND":
							time.sleep(0.1)
						print("Vehicle is in LAND mode")
						send_land_message(x_ang, y_ang)
					else:
						send_land_message(x_ang, y_ang)

					# draw the contour and center of the shape in the image 
					cv2.drawContours(np_data, [c], -1, (0, 0, 255), 2)
					cv2.circle(np_data, (cX, cY), 7, (0,0,255), -1)
					cv2.putText(np_data, "Landing Target", (cX-20, cY-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

					# show the image 
					# cv2.imshow("Image", np_data)
					# cv2.waitKey(0)


		except Exception as e: 
			print("Target not found")
			# print e
			notfound_count += 1

		# the new way using cvbridge
		new_msg = bridge.cv2_to_imgmsg(np_data, encoding="rgb8")
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

def precision_land_landing_pad(vehicle):
	print('Landing...')
	rospy.init_node('drone_node', anonymous=False)
	sub = rospy.Subscriber('/camera/color/image_raw', Image, pl_msg_receiver_landing_pad)
	# rospy.spin()
	while vehicle.armed == True:
		time.sleep(1)
	return None 

def find_letter(letter):

	global vehicle, notfound_count, found_count, time_last, time_to_wait,marker1_found, marker2_found, marker1_id, marker2_id, search_now
	
	if time.time() - time_last > time_to_wait and search_now == True:
		### the old way ###
		# np_data = rnp.numpify(message)  # deserialize image data into array 

		### try cvbridge 
		bridge = CvBridge()
		np_data = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
		# gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
		# gray_img = get_grayscale(np_data)

		# change the image to remove noise 
		image = np_data
		orig = image.copy()
		(origH, origW) = image.shape[:2]
		# set the new width and height and then determine the ratio in change
		# for both the width and height
		(newW, newH) = (320, 320)
		rW = origW / float(newW)
		rH = origH / float(newH)
		# resize the image and grab the new image dimensions
		image = cv2.resize(image, (newW, newH))
		(H, W) = image.shape[:2]

		# define the two output layer names for the EAST detector model that
		# we are interested in -- the first is the output probabilities and the
		# second can be used to derive the bounding box coordinates of text
		layerNames = [
			"feature_fusion/Conv_7/Sigmoid",
			"feature_fusion/concat_3"]
		# load the pre-trained EAST text detector
		print("[INFO] loading EAST text detector...")
		net = cv2.dnn.readNet("frozen_east_text_detection.pb")

		# construct a blob from the image and then perform a forward pass of
		# the model to obtain the two output layer sets
		blob = cv2.dnn.blobFromImage(image, 1.0, (W, H),
			(123.68, 116.78, 103.94), swapRB=True, crop=False)
		net.setInput(blob)
		(scores, geometry) = net.forward(layerNames)
		# decode the predictions, then  apply non-maxima suppression to
		# suppress weak, overlapping bounding boxes
		(rects, confidences) = decode_predictions(scores, geometry)
		boxes = non_max_suppression(np.array(rects), probs=confidences)

		# initialize the list of results
		results = []
		# loop over the bounding boxes
		for (startX, startY, endX, endY) in boxes:
			# scale the bounding box coordinates based on the respective
			# ratios
			startX = int(startX * rW)
			startY = int(startY * rH)
			endX = int(endX * rW)
			endY = int(endY * rH)
			# in order to obtain a better OCR of the text we can potentially
			# apply a bit of padding surrounding the bounding box -- here we
			# are computing the deltas in both the x and y directions
			padding = 0.0
			dX = int((endX - startX) * padding)
			dY = int((endY - startY) * padding)
			# apply padding to each side of the bounding box, respectively
			startX = max(0, startX - dX)
			startY = max(0, startY - dY)
			endX = min(origW, endX + (dX * 2))
			endY = min(origH, endY + (dY * 2))
			# extract the actual padded ROI
			roi = orig[startY:endY, startX:endX]

					# in order to apply Tesseract v4 to OCR text we must supply
			# (1) a language, (2) an OEM flag of 1, indicating that the we
			# wish to use the LSTM neural net model for OCR, and finally
			# (3) an OEM value, in this case, 7 which implies that we are
			# treating the ROI as a single line of text
			roi = get_grayscale(roi)
			roi = thresholding(roi)
			roi = noise_removal(cv2.bitwise_not(roi))
			roi = thin_font(roi)
			config = ("-l eng --oem 1 --psm 10")
			text = pytesseract.image_to_string(roi, config=config)
			# add the bounding box coordinates and OCR'd text to the list
			# of results
			if letter in text:
				results.append(((startX, startY, endX, endY), text))

		# sort the results bounding box coordinates from top to bottom
		results = sorted(results, key=lambda r:r[0][1])
		# loop over the results
		for ((startX, startY, endX, endY), text) in results:
			# display the text OCR'd by Tesseract
			# print("OCR TEXT")
			# print("========")
			# print("{}\n".format(text))
			# strip out non-ASCII text so we can draw the text on the image
			# using OpenCV, then draw the text and a bounding box surrounding
			# the text region of the input image
			text = "".join([c if ord(c) < 128 else "" for c in text]).strip()

		return text, startX, startY, endX, endY, H, W 


def center_letter(letter):
	# this function is designed to take a target point from the an image and get the drone center on the target location
	centered = False 

	while not centered:
		text, startX, startY, endX, endY, H, W = find_letter(letter)

		centerX = (startX + endX) / 2 
		centerY = (startY + endY) / 2 

		gps_location = []
		if 0.95*W/2 <= centerX <= 1.05*W/2 and 0.95*H/2 <= centerY <= 1.05*H/2:
			centered = True 
			return None 


def scan_targets(message):
	global vehicle, notfound_count, found_count, time_last, time_to_wait, marker1_found, marker2_found, marker1_id, marker2_id, search_now
	
	if time.time() - time_last > time_to_wait and search_now == True:
		### the old way ###
		# np_data = rnp.numpify(message)  # deserialize image data into array 

		### try cvbridge 
		bridge = CvBridge()
		np_data = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
		# gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
		# gray_img = get_grayscale(np_data)

		# change the image to remove noise 
		image = np_data
		orig = image.copy()
		(origH, origW) = image.shape[:2]
		# set the new width and height and then determine the ratio in change
		# for both the width and height
		(newW, newH) = (320, 320)
		rW = origW / float(newW)
		rH = origH / float(newH)
		# resize the image and grab the new image dimensions
		image = cv2.resize(image, (newW, newH))
		(H, W) = image.shape[:2]

		# define the two output layer names for the EAST detector model that
		# we are interested in -- the first is the output probabilities and the
		# second can be used to derive the bounding box coordinates of text
		layerNames = [
			"feature_fusion/Conv_7/Sigmoid",
			"feature_fusion/concat_3"]
		# load the pre-trained EAST text detector
		print("[INFO] loading EAST text detector...")
		net = cv2.dnn.readNet("frozen_east_text_detection.pb")

		# construct a blob from the image and then perform a forward pass of
		# the model to obtain the two output layer sets
		blob = cv2.dnn.blobFromImage(image, 1.0, (W, H),
			(123.68, 116.78, 103.94), swapRB=True, crop=False)
		net.setInput(blob)
		(scores, geometry) = net.forward(layerNames)
		# decode the predictions, then  apply non-maxima suppression to
		# suppress weak, overlapping bounding boxes
		(rects, confidences) = decode_predictions(scores, geometry)
		boxes = non_max_suppression(np.array(rects), probs=confidences)

		# initialize the list of results
		results = []
		# loop over the bounding boxes
		for (startX, startY, endX, endY) in boxes:
			# scale the bounding box coordinates based on the respective
			# ratios
			startX = int(startX * rW)
			startY = int(startY * rH)
			endX = int(endX * rW)
			endY = int(endY * rH)
			# in order to obtain a better OCR of the text we can potentially
			# apply a bit of padding surrounding the bounding box -- here we
			# are computing the deltas in both the x and y directions
			padding = 0.0
			dX = int((endX - startX) * padding)
			dY = int((endY - startY) * padding)
			# apply padding to each side of the bounding box, respectively
			startX = max(0, startX - dX)
			startY = max(0, startY - dY)
			endX = min(origW, endX + (dX * 2))
			endY = min(origH, endY + (dY * 2))
			# extract the actual padded ROI
			roi = orig[startY:endY, startX:endX]

					# in order to apply Tesseract v4 to OCR text we must supply
			# (1) a language, (2) an OEM flag of 1, indicating that the we
			# wish to use the LSTM neural net model for OCR, and finally
			# (3) an OEM value, in this case, 7 which implies that we are
			# treating the ROI as a single line of text
			roi = get_grayscale(roi)
			roi = thresholding(roi)
			roi = noise_removal(cv2.bitwise_not(roi))
			roi = thin_font(roi)
			config = ("-l eng --oem 1 --psm 10")
			text = pytesseract.image_to_string(roi, config=config)
			# add the bounding box coordinates and OCR'd text to the list
			# of results
			results.append(((startX, startY, endX, endY), text))

		# sort the results bounding box coordinates from top to bottom
		results = sorted(results, key=lambda r:r[0][1])
		# loop over the results
		for ((startX, startY, endX, endY), text) in results:
			# display the text OCR'd by Tesseract
			# print("OCR TEXT")
			# print("========")
			# print("{}\n".format(text))
			# strip out non-ASCII text so we can draw the text on the image
			# using OpenCV, then draw the text and a bounding box surrounding
			# the text region of the input image
			text = "".join([c if ord(c) < 128 else "" for c in text]).strip()

			if "N" in text:
				print('Found the N')
				# center_letter('N')
				marker2_found = True 

			if "R" in text:
				print('Found the R')
				# center_letter('R')
				marker1_found = True

			if ("O" in text) or ("0" in text) or ("o" in text):
				print('Found the Helipad')  


			output = orig.copy()
			cv2.rectangle(output, (startX, startY), (endX, endY),
				(0, 0, 255), 2)
			cv2.putText(output, text, (startX, startY - 20),
				cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
			# show the output image
			# cv2.imshow("Text Detection", output)

			# the old way 
			# new_msg = rnp.msgify(Image, output, encoding="rgb8")

			# the new way using cvbridge
			new_msg = bridge.cv2_to_imgmsg(output, encoding="rgb8")

			newimg_pub.publish(new_msg)
			# cv2.waitKey(0)


		# # extract any text 
		# d = pytesseract.image_to_data(gray_img, output_type=Output.DICT)
		# print("Image to String Prediction:",pytesseract.image_to_string(gray_img,config='--psm 10'))
		# n_boxes = len(d['text'])

		# # used for the aruco markers 
		# # ids = ''
		# # (corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

		# # try:
		# for i in range(n_boxes):
		# 	if int(d['conf'][i]) > 10:
		# 		# print(text)
		# 		(text, x, y, w, h) = (d['text'][i], d['left'][i], d['top'][i], d['width'][i], d['height'][i])
		# 		if len(text) > 0:
		# 			print("Predicted text:",text, " with confidence:", int(d['conf'][i]))
		# 		# don't show empty text
		# 		if text and text.strip() != "":
		# 			np_data = cv2.rectangle(np_data, (x, y), (x + w, y + h), (0, 255, 0), 2)
		# 			np_data = cv2.putText(np_data, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

		# # except Exception as e: 
		# # 	print('Nothing Found...')
		# # 	return None
		# cv2.imshow('frame', np_data)
		# new_msg = rnp.msgify(Image, np_data, encoding="rgb8")
		# newimg_pub.publish(new_msg)
		time_last = time.time()

	else:
		return None   


def search(vehicle, bound_gps):
	"""A function to search the field for the targets"""
	global marker1_found, marker2_found, search_now 
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
		search_now = True 
		time.sleep(5.0)
		search_now = False 
		if marker1_found==True and marker2_found == True:
			print('Found both Markers, returning to launch...')
			sub.unregister()
			
			return None 

	if (marker2_found == False or marker1_found ==False):
		print('Mission failed, we will get them next time...')
		sub.unregister()

	return None 

############ OpenCV scripts ##############

def thin_font(image):
	image = cv2.bitwise_not (image)
	kernel = np.ones((2,2), np.uint8)
	image = cv2.erode(image, kernel, iterations=3)
	image = cv2.bitwise_not(image)
	return(image)

def noise_removal(image):
	kernel = np.ones((1,1), np.uint8)
	image = cv2.dilate(image, kernel, iterations=1)
	kernel = np.ones((1,1), np.uint8)
	image = cv2.erode(image, kernel, iterations=1)
	image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
	image = cv2.medianBlur(image, 3)
	return image

def get_grayscale(image):
	return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

def thresholding(image):
	return cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

def decode_predictions(scores, geometry):
	# grab the number of rows and columns from the scores volume, then
	# initialize our set of bounding box rectangles and corresponding
	# confidence scores
	(numRows, numCols) = scores.shape[2:4]
	rects = []
	confidences = []
	# loop over the number of rows
	for y in range(0, numRows):
		# extract the scores (probabilities), followed by the
		# geometrical data used to derive potential bounding box
		# coordinates that surround text
		scoresData = scores[0, 0, y]
		xData0 = geometry[0, 0, y]
		xData1 = geometry[0, 1, y]
		xData2 = geometry[0, 2, y]
		xData3 = geometry[0, 3, y]
		anglesData = geometry[0, 4, y]
		# loop over the number of columns
		for x in range(0, numCols):
			# if our score does not have sufficient probability,
			# ignore it
			if scoresData[x] < 0.5:
				continue
			# compute the offset factor as our resulting feature
			# maps will be 4x smaller than the input image
			(offsetX, offsetY) = (x * 4.0, y * 4.0)
			# extract the rotation angle for the prediction and
			# then compute the sin and cosine
			angle = anglesData[x]
			cos = np.cos(angle)
			sin = np.sin(angle)
			# use the geometry volume to derive the width and height
			# of the bounding box
			h = xData0[x] + xData2[x]
			w = xData1[x] + xData3[x]
			# compute both the starting and ending (x, y)-coordinates
			# for the text prediction bounding box
			endX = int(offsetX + (cos * xData1[x]) + (sin * xData2[x]))
			endY = int(offsetY - (sin * xData1[x]) + (cos * xData2[x]))
			startX = int(endX - w)
			startY = int(endY - h)
			# add the bounding box coordinates and probability score
			# to our respective lists
			rects.append((startX, startY, endX, endY))
			confidences.append(scoresData[x])
	# return a tuple of the bounding boxes and associated confidences
	return (rects, confidences)



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
		# precision_land(vehicle)
		precision_land_landing_pad(vehicle)

		# while vehicle.armed == True:
		# 	print('Waiting for disarm...')
		# 	time.sleep(0.1)

	except rospy.ROSInterruptException:
		pass 