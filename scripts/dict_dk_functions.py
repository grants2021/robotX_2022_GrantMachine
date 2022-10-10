import time 
# import exceptions
import math 
import rospy 
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3Stamped
import cv2
import cv2.aruco as aruco
import imutils
from imutils.object_detection import non_max_suppression
import sys
import numpy as np
import pandas as pd
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, APIException
from pymavlink import mavutil    
from array import array 
import os
import pytesseract
from pytesseract import Output
##################  GENERIC FUNCTIONS FOR BASIC MOVEMENT ################
class Drone:
    def __init__(self, vehicle,IDsDictin,LP_Dictin,takeoff_height,spawn_height):
        global alt_reached, marker_heights, last_seen,\
            IDsDict, LP_Dict, takeofftrueheight, landThreshold, LP_search_tag

        print('running init')
        
        # Setting core operational parameters + initializing drone
        landThreshold=0.1
        self.vehicle = vehicle
        self.takeoff_height = takeoff_height
        self.spawn_height = spawn_height
        self.wamv_height = 1.3
        self.drone_height = 0.194923
        self.wamv_landing = False
        self.rate = rospy.Rate(15)
        self.outarray = []
        alt_reached = False
        
        # aruco dict info
        IDsDict = IDsDictin
        LP_Dict = LP_Dictin
        LP_search_tag = False
        # IDsDict marker info format ['size','height','fcount','nfcount','enfcount','xloc','yloc','lastT']
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters_create()

        # camera details
        self.horizontal_res = 1920
        self.vertical_res = 1448
        self.horizontal_fov = 62.2 * (math.pi / 180)  # convert to radians 
        self.vertical_fov = 48.8 * (math.pi / 180)  # convert to radians
        self.dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]  # from the rostopic camera info
        self.camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
        self.np_camera_matrix = np.array(self.camera_matrix)
        self.np_dist_coeff = np.array(self.dist_coeff)

        
        # keep track of time for update on precision landing
        self.time_last = 0.0
        self.time_to_wait = 0.1
        
        # image publisher info
        self.newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

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
    #%% #################   CALCULATIONS   #################
    
    
    def get_true_abs_height(self):
        return self.vehicle.location.global_relative_frame.alt+self.spawn_height
    
    def get_true_QR_relative_height(self):
        #Relative to bottom of drone!
        if self.wamv_landing:
            return self.vehicle.location.global_relative_frame.alt+(self.spawn_height-self.drone_height-self.wamv_height)
        else:
            return self.vehicle.location.global_relative_frame.alt
    
    def localvel_to_globalvel(self, lvx, lvy):
        heading = self.vehicle.heading
        gvx = lvx*math.cos(math.radians(heading)) - lvy*math.sin(math.radians(heading))
        gvy = lvx*math.sin(math.radians(heading)) + lvy*math.cos(math.radians(heading))
        return(gvx,gvy)
    
    def dist_to_latlong(self,target_location):
        dLat_m = target_location[0]/1.113195e5
        dLon_m = target_location[1]/1.113195e5
        dTot_m = math.sqrt((dLon_m**2)+(dLat_m**2))
        return(dLat_m,dLon_m,dTot_m)
    
    def latlong_to_dists(self, targetLocation, currentLocation):
        dLat = (targetLocation.lat - currentLocation.lat)*1.113195e5 
        dLon = (targetLocation.lon - currentLocation.lon)*1.113195e5 
        dTot = math.sqrt((dLon*dLon)+(dLat*dLat))
        return(dLat,dLon,dTot)
    
    def local_dist_calc(self, targetLocation, currentLocation):
        dNorth = (targetLocation.north - currentLocation.north)
        dEast = (targetLocation.east - currentLocation.east)
        dDown = (targetLocation.down - currentLocation.down)
        dTot = math.sqrt(dNorth*dNorth+dEast*dEast+dDown*dDown)
        return(dNorth,dEast,dDown,dTot)
        
    
    
    #%% #################   COMPUTER VISION #################
    def avg_to_ang(self,x,y):
        x_ang = (x - self.horizontal_res*0.5)*(self.horizontal_fov/self.horizontal_res)
        y_ang = (y - self.vertical_res*0.5)*(self.vertical_fov/self.vertical_res)
        return(x_ang,y_ang)
    
    def ShapeDetector(self,c):
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
    
    def thin_font(self, image):
        image = cv2.bitwise_not (image)
        kernel = np.ones((2,2), np.uint8)
        image = cv2.erode(image, kernel, iterations=3)
        image = cv2.bitwise_not(image)
        return(image)

    def noise_removal(self, image):
        kernel = np.ones((1,1), np.uint8)
        image = cv2.dilate(image, kernel, iterations=1)
        kernel = np.ones((1,1), np.uint8)
        image = cv2.erode(image, kernel, iterations=1)
        image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
        image = cv2.medianBlur(image, 3)
        return image

    def get_grayscale(self, image):
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def thresholding(self, image):
        return cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    
    def decode_predictions(self, scores, geometry):
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
    
    def scan_targets(self, message):
        global vehicle, notfound_count, found_count, time_last, time_to_wait, marker1_found, marker2_found, marker1_id, marker2_id, search_now
        
        if time.time() - time_last > time_to_wait and search_now == True:
            np_data = rnp.numpify(message)  # deserialize image data into array 
            gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
            # change the image to remove noise 
            image = gray_img
            orig = gray_img.copy()
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
            (rects, confidences) = self.decode_predictions(scores, geometry)
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
                roi = self.get_grayscale(roi)
                roi = self.thresholding(roi)
                roi = self.noise_removal(cv2.bitwise_not(roi))
                roi = self.thin_font(roi)
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
                new_msg = rnp.msgify(Image, output, encoding="rgb8")

                # the new way using cvbridge
                #new_msg = bridge.cv2_to_imgmsg(output, encoding="rgb8")

                self.newimg_pub.publish(new_msg)
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
            #     if int(d['conf'][i]) > 10:
            #         # print(text)
            #         (text, x, y, w, h) = (d['text'][i], d['left'][i], d['top'][i], d['width'][i], d['height'][i])
            #         if len(text) > 0:
            #             print("Predicted text:",text, " with confidence:", int(d['conf'][i]))
            #         # don't show empty text
            #         if text and text.strip() != "":
            #             np_data = cv2.rectangle(np_data, (x, y), (x + w, y + h), (0, 255, 0), 2)
            #             np_data = cv2.putText(np_data, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

            # # except Exception as e: 
            # #     print('Nothing Found...')
            # #     return None
            # cv2.imshow('frame', np_data)
            # new_msg = rnp.msgify(Image, np_data, encoding="rgb8")
            # newimg_pub.publish(new_msg)
            self.time_last = time.time()

        else:
            return None  
        
    def platform_scan(self, message):
        global LP_Dict, LP_search_tag
        if LP_search_tag:
            np_data = rnp.numpify(message)  # deserialize image data into array 
            gray = self.get_grayscale(np_data)
            thresh = self.thresholding(gray)
            thresh = self.noise_removal(thresh)
            # find the contours in the thresholded image 
            cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts=imutils.grab_contours(cnts)
            target_shape = "circle"
            try:
                for c in cnts:
                    # compute the center of the contour
                    shape = self.ShapeDetector(c)
    
                    if shape == target_shape:
                        M=cv2.moments(c)
                        cX = int(M['m10']/M['m00'])
                        cY = int(M['m01']/M["m00"])
    
                        # draw the contour and center of the shape in the image 
                        cv2.drawContours(np_data, [c], -1, (0, 0, 255), 2)
                        cv2.circle(np_data, (cX, cY), 7, (0,0,255), -1)
                        cv2.putText(np_data, "Landing Target", (cX-20, cY-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                print('X loc {}   Y loc {}'.format(cX,cY))
                
    
            except Exception as e:
                print(e)
                pass
    
            new_msg = rnp.msgify(Image, np_data, encoding="rgb8")
            self.newimg_pub.publish(new_msg)
            self.time_last = time.time()
        else:
            pass
    #%% #################   ARMING AND TAKEOFF   #################
    
    
    def arm(self):
        while self.vehicle.is_armable!=True:
            try:
                print("Waiting for vehicle to become armable.")
                time.sleep(2)
            except rospy.ROSInitException or Exception:
                break
            
        print("Vehicle is now armable")
        
        self.vehicle.mode = "GUIDED"
        time.sleep(.5)     
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
            print("Current Altitude: %d"%self.get_true_abs_height())
            print('Goal Alt: %d'%self.takeoff_height)
            if self.get_true_abs_height()>=0.95*self.takeoff_height:
                print("target altitude reached")
                alt_reached = True
                break 
            time.sleep(1)

        return None 

    def return_to_launch(self, home):
        self.vehicle.mode = "RTL"
    
        while self.vehicle.mode != "RTL":
            time.sleep(0.1)
    
        distanceToTargetLocation = self.latlong_to_dists(home, self.vehicle.location.global_relative_frame)
        currentDistance = distanceToTargetLocation
        
        while currentDistance > distanceToTargetLocation* 0.01:
            time.sleep(0.1)
            currentDistance = self.latlong_to_dists(home, self.vehicle.location.global_relative_frame)
        # when the drone is within close proximity to the home spot, switch to landin
        self.vehicle.mode = "LAND"
    
        while self.vehicle.mode != "LAND":
            time.sleep(0.1)
        return None 



    #%% #################   HORIZONTAL MOVEMENT   #################
    # Use print(msg.__str__()) for message printout to console
    
    def goto(self, targetLocation): #Pretty useless, uses dronekit movement instead of MAVLINK 
        # input is a LocationGlobalRelative variable
        distanceToTargetLocation = self.latlong_to_dists(targetLocation, self.vehicle.location.global_relative_frame)[-1]

        self.vehicle.simple_goto(targetLocation)

        while self.vehicle.mode.name == "GUIDED":
            currentDistance = self.latlong_to_dists(targetLocation, self.vehicle.location.global_relative_frame)[-1]
            if currentDistance < distanceToTargetLocation*0.01:
                print("Reached target waypoint")
                time.sleep(2)
                break 
            time.sleep(1)
        return None

    # go to a positional command with a +x being the heading of the drone 
    def goto_position_target_local_ned(self, north, east, down):
        startloc = self.vehicle.location.local_frame
        disttotarget = math.sqrt(north*north+east*east+down*down)
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,          # time_boot_ms (not used)
            0, 0,       # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            east, -north, down,  # position
            0, 0, 0,    # velocity 
            0, 0, 0,    # acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)       # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)
        print(msg.__str__())
        #curloc = self.vehicle.location.local_frame
        #currentDistance = self.local_dist_calc(curloc,startloc)[-1]
        #while currentDistance < disttotarget*0.01:
        #    self.rate.sleep()
        #    currentDistance = self.local_dist_calc(self.vehicle.location.local_frame,startloc)[-1]
        return None
    
    # send a velocity command with a +x being the heading of the drone 
    def send_local_ned_velocity(self, vx, vy, vz):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,          # time_boot_ms (not used)
            0, 0,       # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111,  # type_mask (only velocities enabled)
            0, 0, 0,    # position
            vx, vy, vz, # velocity 
            0, 0, 0,    # acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)       # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)
        
        #self.rate.sleep()
        stime = time.time()
        print(self.vehicle.location.local_frame)
        print('--------')
        while time.time()-stime < 5:
            print(self.vehicle.location.local_frame)
        return None
    
    # send a velocity command with +x being the true north of earth 
    def send_global_ned_velocity(self, vx, vy, vz):
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,          # time_boot_ms (not used)
            0, 0,       # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame  
            0b0000111111000111,  # type_mask (only velocities enabled)
            0, 0, 0,    # position
            vx, vy, vz, # velocity 
            0, 0, 0,    # acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)       # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)
        self.rate.sleep()
        return None

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
        self.vehicle.send_mavlink(msg)
        self.rate.sleep()
        return None
    
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
        self.vehicle.send_mavlink(msg)
        self.rate.sleep()
        return None
    
    def LP_search(self, bound_gps):
        global LP_Dict, LP_search_tag
        sub = rospy.Subscriber('/camera/color/image_raw', Image, self.platform_scan)

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
            self.goto(vehicle, LocationGlobalRelative(point[0], point[1], self.takeoff_height))
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
    
    def simple_look(self):
        sub = rospy.Subscriber('/camera/color/image_raw', Image, self.platform_scan)
    #%% #################   COMMUNICATION   #################
    
    
    def wamv_vel_callback(self, data): #Translating data from WAM-V publisher
        if type(data) == NavSatFix:
            self.outarray = np.array([0.0]*3)
            self.outarray[0] = data.latitude
            self.outarray[1] = data.longitude
            self.outarray[2] = data.header.stamp.secs
        elif type(data) == Vector3Stamped:
            self.outarray = np.array([0.0]*4)
            self.outarray[0] = data.vector.x
            self.outarray[1] = data.vector.y
            self.outarray[2] = data.vector.z
            self.outarray[3] = data.header.seq
        elif type(data) == Twist:
            self.outarray = np.array([0.0]*6)
            self.outarray[0] = data.linear.x
            self.outarray[1] = data.linear.y
            self.outarray[2] = data.linear.z
            self.outarray[3] = data.angular.x
            self.outarray[4] = data.angular.y
            self.outarray[5] = data.angular.z
        return self.outarray
    
    def get_wamv_info(self,desired): #Request function for WAM-V data
        prevarray = self.outarray
        if desired == 'wamv_local_vel':
            tempsub = rospy.Subscriber('cmd_vel', Twist, self.wamv_vel_callback)
        elif desired == 'wamv_global_pos':
            tempsub = rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, self.wamv_vel_callback)
        elif desired == 'wamv_global_vel':
            tempsub = rospy.Subscriber('/wamv/sensors/gps/gps/fix_velocity', Vector3Stamped, self.wamv_vel_callback, queue_size=1)
        if len(prevarray) == 0:
            while len(self.outarray)==0:
                self.rate.sleep()
        else:
            while prevarray[-1]==self.outarray[-1] or len(self.outarray)==0:
                self.rate.sleep()
        tempsub.unregister()
        del tempsub        
        return self.outarray
    
    
    def wamv_vel_adder(self, vx, vy, vz): #Adds WAM-V velocity + yaw to drone velocity
        gvx, gvy = self.localvel_to_globalvel(vx,vy)
        velarray = self.get_wamv_info('wamv_global_vel')
        wamv_vx = velarray[0]
        wamv_vy = -velarray[1]
        wamv_yaw = np.sign(wamv_vy)*math.atan2(vx,vy)
        vx = gvx + wamv_vx
        vy = gvy + wamv_vy
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,
            0,0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 
            #0b0000111100011111,  # .. BITMASK -> consider only the vz ax xy 
            #0b0000111111000111,  # .. BITMASK -> consider only the vx vy vz
            0b0000110111000111,  # .. BITMASK -> consider only the dx dy vz
            0, 0, 0,  # position
            vx, vy, vz,  # velocity 
            0, 0, 0,  # acceleration
            wamv_yaw, 0)
        #print(msg.__str__())
        self.vehicle.send_mavlink(msg)
        self.rate.sleep()
        
    def wamv_vel_follower(self,vz): #Used for when no drone velocity desired
        self.wamv_vel_adder(0, 0, vz)
    
    
    #%% #################   LANDING   #################
    
    
    def return_for_land(self,max_vel):
        global IDsDict
        dist = [1,1,1]
        while dist[-1]>=1:
            try:
                vels = [0,0]
                uav_pos = self.vehicle.location.global_relative_frame
                wamv_pos = self.get_wamv_info('wamv_global_pos')
                dist = self.latlong_to_dists(wamv_pos,uav_pos)
                scale_base = np.argmax(dist[0:2])
                not_base = np.argmin(dist[0:2])
                vels[scale_base] = max_vel
                vels[not_base] = max_vel*dist[not_base]/dist[scale_base]
                self.wamv_vel_adder(vels[0],vels[1],0)
            except rospy.ROSInitException or APIException or KeyboardInterrupt or SystemExit:
                pass
    
    def velLand(self,x_ang,y_ang,zreal,zoffset):
        vland = self.vehicle.parameters['LAND_SPEED']/100
        tland = zoffset/vland
        dx = math.atan(x_ang)*zreal
        dy = math.atan(y_ang)*zreal
        vx = dx/tland
        vy = dy/tland
        print('Marker at x:',dx,' y:',dy,' z:',zoffset)
        if self.wamv_landing:
            self.wamv_vel_adder(vx,vy,vland)
        else:
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
            #print(msg.__str__())
            self.vehicle.send_mavlink(msg)
            self.rate.sleep()
    
    def dvel_based_landing(self,landThreshold):
        z = self.get_true_QR_relative_height()
        vz = self.vehicle.parameters['LAND_SPEED']/300
        vz_real = vz
        count = 0
        while vz_real>0 and count<5:
            self.wamv_vel_adder(0, 0, vz)
            self.rate.sleep()
            z = self.get_true_QR_relative_height()
            vz_real = self.vehicle.velocity[-1]
            print('LANDING: Vi:{:0.5f} Vr:{:0.5f} Height:{:0.5f}'.format(vz,vz_real,z))
            if vz_real<=0.0:
                count +=1
        
        for i in range(0,20):
            self.wamv_vel_adder(0, 0, 0)
            self.rate.sleep()
            print('Holding...')
            
        print('Going into LAND mode')
        if self.vehicle.armed==True:
            self.vehicle.armed=False
            while self.vehicle.armed==True:
                try:
                    time.sleep(1)
                except rospy.ROSInterruptException or APIException or KeyboardInterrupt or SystemExit:
                    pass
            print("Vehicle is in LAND mode")
        time.sleep(1)
        return True
    
    def tapered_landing(self,landThreshold):
        z = self.get_true_QR_relative_height()
        zratio=1
        vz = 1
        while zratio>0.02:
            print(z)
            zratio = (np.exp(1.31*z/landThreshold)/(np.exp(1)))-0.36
            vz = zratio*self.vehicle.parameters['LAND_SPEED']/100
            self.wamv_vel_adder(0, 0, vz)
            z = self.get_true_QR_relative_height()
            #count += 1
        print('Finished tapered landing, waiting to disarm')
        if self.vehicle.armed==True:
            self.vehicle.armed=False
            while self.vehicle.armed==True:
                self.rate.sleep()
            print("Vehicle is disarmed")
        
        '''
        print('Finished tapered landing,  mode')
        if self.vehicle.mode !="LAND":
            self.vehicle.mode = VehicleMode("LAND")
            while self.vehicle.mode != "LAND":
                self.wamv_vel_adder(0, 0, 0)
                #self.rate.sleep()
            print("Vehicle is in LAND mode")
        '''
        return True
        
    def landScript(self,x_avg,y_avg,z,zoffset):
        global landThreshold
        shutdowntag = False
        #Coordinate system local 
        #Camera +X is local East, Camera +Y is local North
        #MAVLINK wants +X to be local North and +Y to be local East
        y_ang, x_ang = self.avg_to_ang(x_avg, y_avg)
        x_ang = -x_ang
        
        if self.get_true_QR_relative_height() <= landThreshold:
            shutdowntag = self.dvel_based_landing(landThreshold)
            if not shutdowntag:
                print('Drone did not complete shutdown fully')
            return True
        else:
            if zoffset<1:
                zoffset=z   
            self.velLand(x_ang,y_ang,z,zoffset)
            return False
        
        #print('Sent Land Message at x_ang=%0.5f'%x_ang+' y_ang=%0.5f'%y_ang)
    
    def rerouteScript(self,x_avg,y_avg,z,desiredZ):
        dz = desiredZ-z
        y_ang, x_ang = self.avg_to_ang(x_avg, y_avg)
        x_ang = -x_ang
        dx = math.atan(x_ang)*dz
        dy = math.atan(y_ang)*dz
        d_max = np.max([dx,dy,dz])
        vx = dx/d_max
        vy = dy/d_max
        vz = dz/d_max
        
        print('Reroute Marker at x:',dx,' y:',dy,' z:',z)
        
        if self.wamv_landing:
            gvx, gvy = self.localvel_to_globalvel(vx,vy)
            velarray = self.get_wamv_info('wamv_global_vel')
            wamv_vx = velarray[1]
            wamv_vy = -velarray[1]
            vx = gvx + wamv_vx
            vy = gvy + wamv_vy
            msg = self.vehicle.message_factory.set_position_target_global_int_encode(
                0,
                0,0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 
                #0b0000111100011111,  # .. BITMASK -> consider only the vz ax xy 
                #0b0000111111000111,  # .. BITMASK -> consider only the vx vy vz
                0b0000111111000111,  # .. BITMASK -> consider only the dx dy vz
                0, 0, 0,  # position
                vx, vy, vz,  # velocity 
                0, 0, 0,  # acceleration
                0, 0)
        else:
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0,0,
                mavutil.mavlink.MAV_FRAME_BODY_NED, 
                #0b0000111100011111,  # .. BITMASK -> consider only the vz ax xy 
                #0b0000111111000111,  # .. BITMASK -> consider only the vx vy vz
                0b0000111111000111,  # .. BITMASK -> consider only the dx dy vz
                0, 0, 0,  # position
                vx, vy, vz,  # velocity 
                0, 0, 0,  # acceleration
                0, 0)
        #print(msg.__str__())
        self.vehicle.send_mavlink(msg)
        
    
    def landing_receiver(self, message):
        global IDsDict, alt_reached, landThreshold
        
        #If drone shutdown flag is triggered
        if self.shutdowntag: 
            #print('Mission Completed')
            #shutdowntag = True
            count = 0 
            if self.vehicle.armed==True:
                self.vehicle.armed = False
                while self.vehicle.armed==True or count<=20 or not rospy.is_shutdown():
                    try:    
                        print("Waiting for vehicle to disarm.")
                        time.sleep(1)
                        count += 1
                    except rospy.ROSInitException or Exception:
                        print('Vehicle disarmed')
                        break
            return self.shutdowntag
        
        #If drone is ready for next cmd
        elif time.time() - self.time_last > self.time_to_wait and alt_reached:
            np_data = rnp.numpify(message)  # deserialize image data into array 
            gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
            idstemp = []
            (corners, idstemp, rejected) = aruco.detectMarkers(image=gray_img, dictionary=self.aruco_dict, parameters=self.parameters)
            land_id_to_find = 0
            for i in range(0,len(IDsDict.columns)): #Scanning image for markers
                curalt = self.get_true_QR_relative_height()
                if curalt<=IDsDict.loc['height'].iloc[i]:
                    land_id_to_find = IDsDict.columns[i]
                    marker_size = IDsDict.loc['size'].iloc[i]
                    #print('working----',land_id_to_find)
                    if i!=0: #If marker is not final landing adjust height to next smallest h threshold
                        offset = curalt-IDsDict.loc['height'].iloc[i-1]
                    else: #If final marker set heigh goal to landing threshold
                        offset = curalt-landThreshold
                    break
            
            #If drone is above all marker thresholds but some markers are found look for largest marker
            if not land_id_to_find and idstemp is not None:
                land_id_to_find = IDsDict.columns[-1]
                marker_size = IDsDict.loc['size'].iloc[-1]
                offset = curalt - IDsDict.loc['height'].iloc[-2] 
            
            #If drone is above all marker threshholds and no markers are found go down to largest marker height
            if not land_id_to_find and idstemp is None:
                land_id_to_find = IDsDict.columns[-1]
                marker_size = IDsDict.loc['size'].iloc[-1]
                deltaz = IDsDict.loc['height'].iloc[-1] - curalt
                print('Going Down '+str(deltaz)+' to '+str(IDsDict.loc['height'].iloc[-1]))
                self.wamv_vel_follower(self.vehicle.parameters['LAND_SPEED']/100)
            
            #Landing script starts
            try:
                #If any markers are found
                if idstemp is not None:
                    ids=list()
                    for X in idstemp:
                        ids.append(X[0])

                    rvec = [list()]*2
                    tvec = [list()]*2
                    #Update marker dictonary with newest positional information
                    for i in range(0,len(IDsDict.columns)):
                        if IDsDict.columns[i] in ids:
                            k = np.where(np.array(ids)==IDsDict.columns[i])[0][0]
                            #corners = cornerstemp[0][k]
                            
                            ret = aruco.estimatePoseSingleMarkers(corners[k], IDsDict.loc['size'].iloc[i], \
                                cameraMatrix=self.np_camera_matrix,distCoeffs=self.np_dist_coeff)
                            (rvec[i], tvec[i]) = (ret[0][0,0,:], ret[1][0,0,:])  # rotation vctr and translation vctr                                 
                            #xerror = '{:.2f}'.format(tvec[i][0])  # x error 
                            #yerror = '{:.2f}'.format(tvec[i][1])
                            #zerror = '{:.2f}'.format(tvec[i][2])
            
                            xcorns = [corners[k][0][0][0],corners[k][0][1][0],corners[k][0][2][0],corners[k][0][3][0]]
                            ycorns = [corners[k][0][0][1],corners[k][0][1][1],corners[k][0][2][1],corners[k][0][3][1]]
                            
                            # Updating dictonary with recent relative positions
                            IDsDict.loc['xloc'].iloc[i] = sum(xcorns)/4 
                            IDsDict.loc['yloc'].iloc[i] = sum(ycorns)/4
                            IDsDict.loc['lastT'].iloc[i] = time.time()
                    
                    #If target marker found update found count and start descending
                    if land_id_to_find in ids:
                        IDsDict[land_id_to_find].loc['fcount'] += 1
                        id_to_find = land_id_to_find
                        reroutetag = False
                        landtag = True
                    
                    #If target marker not found
                    else:
                        IDsDict[land_id_to_find].loc['nfcount'] += 1
                        deltaT = time.time()-IDsDict[land_id_to_find].loc['lastT']
                        timelim = 2
                        #If target marker was recently lost continue using last known pos
                        if deltaT < timelim:
                            print("Target Not Found: Using last pos found "+\
                                  '%0.5f seconds ago'%deltaT)
                            id_to_find = land_id_to_find
                            reroutetag = False
                            landtag = True
                        #If target was lost 'timelim' time ago reroute to smallest found marker
                        else: 
                            id_to_find = ids[0]
                            reroutetag = True
                            landtag = False
                            print("Target Not Found: Heading to ID ",id_to_find)
                    
                    #Get target marker locations
                    k2 = np.where(IDsDict.columns==id_to_find)[0][0]
                    x_avg = IDsDict[id_to_find].loc['xloc']
                    y_avg = IDsDict[id_to_find].loc['yloc']
                    curalt = self.get_true_QR_relative_height()
                    marker_position = 'Marker Position: x='+str(x_avg)+' y='+str(y_avg)+' z='+str(curalt)+''
                    
                    if landtag:
                        print('Target Found - Landing @ ID ',id_to_find)
                        self.shutdowntag = self.landScript(x_avg,y_avg,curalt,offset)
                        self.rate.sleep()
                    
                    #Need to work on this...
                    if reroutetag:
                        desiredZ = IDsDict[land_id_to_find].loc['height']
                        self.rerouteScript(x_avg,y_avg,curalt,desiredZ)
                        self.rate.sleep()
                        
                    #Overlay marker details in camera publisher
                    aruco.drawDetectedMarkers(np_data, corners)
                    aruco.drawAxis(np_data, self.np_camera_matrix, self.np_dist_coeff, rvec[k2], tvec[k2], 10)  
                    cv2.putText(np_data, marker_position, (10,50), 0, 0.7, (255, 0,0), thickness=2)
                    #print('FOUND COUNT:'+str(IDsDict.columns.values)+str(IDsDict.loc['fcount'].values)\
                    #      +" NOT FOUND COUNT:"+str(IDsDict.columns.values)+str(IDsDict.loc['nfcount'].values)+'\n\n')
                    self.rate.sleep() 
                
                #If no markers are found
                else: 
                    IDsDict[land_id_to_find].loc['nfcount'] += 1
            
            #If something goes wrong and breaks loop
            except Exception as e: 
                #print("Target not found")
                print(e)
                IDsDict[land_id_to_find].loc['nfcount'] += 1 
            
            #Take new photo and make next message
            new_msg = rnp.msgify(Image, np_data, encoding="rgb8")
            self.newimg_pub.publish(new_msg)
            self.time_last = time.time()
        else:
            return None 
    
    
    def landing_subscriber(self,IDsDictin,landflag):
        global IDsDict
        IDsDict = IDsDictin
        self.shutdowntag = False
        while not rospy.is_shutdown() or not self.shutdowntag:
            try:
                if landflag == 0:
                    self.wamv_landing = False
                elif landflag == 1:
                    self.wamv_landing = True
                sub = rospy.Subscriber('/camera/color/image_raw', Image, self.landing_receiver)
            except rospy.ROSInitException or APIException or KeyboardInterrupt or SystemExit:
                break
        print('Landing stopped')
        return True
        #sys.exit()
        #rospy.spin()
     
    
#%% 
#################   QUICK LAUNCH   #################

username = os.getlogin()

def launcher(launchID, qrttag, launchName):
    #assumedWAMV = 
    if 'drone_wamv' in launchName or 'wamv_drone' in launchName:
        os.system('xdotool key "ctrl+shift+t";xdotool type "roslaunch robotx_2022 wamv_basic_commands.launch";xdotool key Return')
        time.sleep(.3)
    if launchID == 0:
        os.system('xdotool key "ctrl+shift+t";xdotool type "~/ardupilot/Tools/autotest/sim_vehicle.py --console -v ArduCopter";xdotool key Return')
        os.system('xdotool key "ctrl+shift+t";xdotool type "~/QGroundControl.AppImage";xdotool key Return')
    elif launchID == 1:
        os.system('xdotool key "ctrl+shift+t";xdotool type "~/ardupilot/Tools/autotest/sim_vehicle.py -f gazebo-iris --console -v ArduCopter";xdotool key Return')
        time.sleep(.6)
        os.system('xdotool key "ctrl+shift+t";xdotool type "roslaunch robotx_2022 '+launchName+' -user:='+os.getlogin()+'";xdotool key Return')
    elif launchID==2:
        os.system('xdotool key "ctrl+shift+t";xdotool type "~/ardupilot/Tools/autotest/sim_vehicle.py -f gazebo-iris --console -v ArduCopter";xdotool key Return')
        time.sleep(.6)
    if qrttag:
        os.system('xdotool key "ctrl+shift+t";xdotool type "rqt";xdotool key Return')
        time.sleep(.3)
    
    os.system('xdotool key "alt+1"')
    time.sleep(.3)
    