#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 14:24:06 2022

@author: grant
"""

#!/usr/bin/env python3
# license removed for brevity

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension, Bool
import numpy as np
import math
import numpy
import sys
import time
import threading as th

class Node():
    def __init__(self):
        self.linear_scaling = rospy.get_param('~linear_scaling',0.2)
        self.angular_scaling = rospy.get_param('~angular_scaling',0.05)
        self.thrust_angle_speed = 0.1
        self.max_angle = rospy.get_param("~max_angle", math.pi / 2)
        self.rate = rospy.Rate(15)
        
        #self.lin_data_pub = rospy.Publisher("/wamv/linear",Float32MultiArray,queue_size=10)
        #self.usv_data_pub = rospy.Publisher("/wamv/data",Float32MultiArray,queue_size=10)
        self.left_ang_pub = rospy.Publisher('left_thrust_angle', Float32, queue_size=1)
        self.right_ang_pub = rospy.Publisher('right_thrust_angle', Float32, queue_size=1)
        self.left_cmd_pub = rospy.Publisher("left_thrust_cmd",Float32,queue_size=10)
        self.right_cmd_pub = rospy.Publisher("right_thrust_cmd",Float32,queue_size=10)
        self.vel_cmd_pub = rospy.Publisher("/wamv_vel_cmd",Float32MultiArray,queue_size=10)
        
        self.left_msg = Float32()
        self.right_msg = Float32()
        self.angle_msg = Float32()
        
        self.vel_cmd = Float32MultiArray()
        self.vel_cmd.layout.dim.append(MultiArrayDimension())
        self.vel_cmd.layout.dim[0].label = 'Vel Commands' # x y z th speed turn
        self.vel_cmd.layout.dim[0].size = 6
        self.vel_cmd.layout.dim[0].stride = 1
        self.vel_cmd.layout.data_offset = 0
        self.vel_cmd.data = [0]*(6)
        
        
    def constant(self,command):
        while not rospy.is_shutdown():
            curr_angle = self.thrust_angle_speed*np.sign(command[2])
            curr_angle = numpy.clip(curr_angle,
                                    -self.max_angle, self.max_angle).item()
            # Publish thrust angle
            angle_msg = Float32()
            angle_msg.data = curr_angle
            self.left_ang_pub.publish(angle_msg)
            self.right_ang_pub.publish(angle_msg)
            
            self.right_msg = Float32(command[0])
            self.left_msg = Float32(command[1])
            
            
            self.right_cmd_pub.publish(self.right_msg)
            self.left_cmd_pub.publish(self.left_msg)
            print('Sending right ',self.right_msg,' left ',self.left_msg)
            self.rate.sleep()

    def burst(self,command):
        for i in range(0,1):
            curr_angle = self.thrust_angle_speed*np.sign(command[2])
            curr_angle = numpy.clip(curr_angle,
                                    -self.max_angle, self.max_angle).item()
            # Publish thrust angle
            angle_msg = Float32()
            angle_msg.data = curr_angle
            self.left_ang_pub.publish(angle_msg)
            self.right_ang_pub.publish(angle_msg)
            
            self.right_msg = Float32(command[0])
            self.left_msg = Float32(command[1])
            
            
            self.right_cmd_pub.publish(self.right_msg)
            self.left_cmd_pub.publish(self.left_msg)
            self.rate.sleep()
    
    def send_vels(self,input_vel_cmds):
        #while not rospy.is_shutdown():
        self.vel_cmd.data = list(input_vel_cmds)
        self.vel_cmd_pub.publish(self.vel_cmd)
        self.rate.sleep()
        
            
    def twist_callback(self,data):
        #print(data)
        self.left_msg.data = data.linear.x
        self.right_msg.data = data.linear.x
        self.left_msg.data += -1*data.angular.z
        self.right_msg.data += data.angular.z
        #print('Sending right ',self.right_msg,' left ',self.left_msg)
        self.right_cmd_pub.publish(self.right_msg)
        self.left_cmd_pub.publish(self.left_msg)
        

    def subscriber(self,iternum):
        count = 0
        #time.sleep(3)
        print('waiting to connect...')
        #if iternum == 0:
        #    while not rospy.is_shutdown():
        #        sub = rospy.Subscriber('cmd_vel',Twist,self.twist_callback)
        #else:
        #    while count <= iternum:
        #        sub = rospy.Subscriber('cmd_vel',Twist,self.twist_callback)
        #        count += 1
        #sys.exit()
    
    def takeoff_callback(self, takeoff_bool):
        global timer, takeofftag
        deltaT = time.perf_counter()-timer
        if takeofftag == 0:
            #print(takeoff_bool.data)
            if takeoff_bool.data:
                takeofftag = 1
            elif deltaT >= 180*10**1:
                takeofftag = 2
            else:
                takeofftag = 0
            time.sleep(.5)
            
    def empty_callback(self, data):
        pass
    
    def twist_test_callback(self, data):
        global cmd_vel_tag
        cmd_vel_tag = data.data
        #print(data)
        if cmd_vel_tag:
            print('Connected to cmd_vel!')
            time.sleep(0.5)
        else:
            print('Attempting to connect to cmd_vel...')
            time.sleep(1)
            

if __name__ == "__main__":
    global timer, takeofftag, cmd_vel_tag
    rospy.init_node('wamv_dk_functions', anonymous=False)
    node = Node()
    iter_num = rospy.get_param("~iter_num",1)
    input_vel_cmds = rospy.get_param("~input_vel_cmds",[1.0,0.0,0.0,0.0,.1,0.0])
    if type(input_vel_cmds[0])==str:
        temp_vel_cmds = input_vel_cmds.split(' ')
        input_vel_cmds = [float(x) for x in temp_vel_cmds]
    timer = time.perf_counter()
    #node.subscriber(0)
    sintag = False
    takeofftag = 0
    cmd_vel_tag = 0
    print('waiting to connect...')
    i = 0
    while not rospy.is_shutdown():
        takeoff_sub = rospy.Subscriber('/drone/takeoff_tag',Bool,node.takeoff_callback,queue_size=1)
        #print(takeofftag)
        if i == 4:
            print("Waiting for drone to take off")
        rospy.sleep(0.5)
        i += 1
        if takeofftag !=0:
            print('Drone has taken off')
            takeoff_sub.unregister()
            break
        
    if takeofftag == 1:
        print("Sending WAMV Velocities")
        amplitude = input_vel_cmds[-1]
        starttime = time.time()
        while not rospy.is_shutdown():
            if sintag==True:
                input_vel_cmds[-1] = amplitude*math.sin(1*math.pi*(time.time()-starttime)/60)
            node.send_vels(input_vel_cmds)
            twist_sub = rospy.Subscriber('cmd_vel',Twist,node.twist_callback)
    elif takeofftag == 2:
        print('Drone did not take off after 90 seconds')
    else:
        sys.exit()
        print('idk what went wrong')