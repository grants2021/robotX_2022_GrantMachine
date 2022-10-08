#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 20:45:40 2022

@author: grant
"""
from __future__ import print_function

import threading

#import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, Bool

import sys, select, termios, tty

msg = ""

class PublishThread(threading.Thread):
    def __init__(self, rate):
        global cmd_callback_out
        cmd_callback_out = [0.0]*6
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.pub_trigger = rospy.Publisher('cmd_vel_trigger', Bool, queue_size = 10)
        self.pub_trigger.publish(Bool(False))
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            self.pub_trigger.publish(Bool(True))
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        #print('Updating to: ',self.x,self.y,self.z,self.th,self.speed,self.turn)
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)
    
    def cmdcallback(crap,data):
        global cmd_callback_out
        cmd_callback_out = list(data.data)
        #print(data.data)
        #return float(data.data)
        #a=2


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    global cmd_callback_out
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('boat_twist')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 0.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None
    
    
    pub_thread = PublishThread(repeat)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    
    try:     
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)
        while not rospy.is_shutdown():
            vel_cmds_sub = rospy.Subscriber('/wamv_vel_cmd',Float32MultiArray,pub_thread.cmdcallback)
            vel_cmds = cmd_callback_out
            
            if vel_cmds:
                #print(vel_cmds)
                x = vel_cmds[0]
                y = vel_cmds[1]
                z = vel_cmds[2]
                th = vel_cmds[3]
                speed = vel_cmds[4]
                turn = vel_cmds[5]
            
            else:
                print('setting 0s')
                x = 0
                y = 0
                z = 0
                th = 0
                speed = 0
                turn = 0
 
            pub_thread.update(x, y, z, th, speed, turn)
    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print('Boat_twist ending')