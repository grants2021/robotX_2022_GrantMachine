#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 14:24:06 2022

@author: grant
"""

#!/usr/bin/env python3
# license removed for brevity

import rospy
from std_msgs.msg import Bool
import threading
import sys

class PublishThread(threading.Thread):
    def __init__(self, ratein):
        super(PublishThread, self).__init__()
        self.takeoff_pub = rospy.Publisher('/drone/takeoff_tag',Bool,queue_size=1, latch=True)
        self.condition = threading.Condition()
        self.done = False
        self.tag = Bool()
        self.tag.data = False
        #self.rate = rospy.Rate(ratein)
        if ratein != 0.0:
            self.timeout = 1.0 / ratein
        else:
            self.timeout = None

        self.start()
    
    def update(self, inputtag):
        self.condition.acquire()
        self.tag.data = inputtag
        self.condition.notify()
        self.condition.release()
    
    def stop(self):
        self.done = True
    
    def run(self):
        while not self.done and not rospy.is_shutdown():
            self.condition.acquire()
            self.condition.wait(self.timeout)
            self.condition.release()
            self.takeoff_pub.publish(self.tag)
            #self.rate.sleep()
        self.tag.data = False
        self.takeoff_pub.publish(self.tag)
        self.stop()

def startup(repeat):
    #rospy.init_node('takeoff_tag', anonymous=False)
    pub_thread = PublishThread(repeat)
    return pub_thread


if __name__ == "__main__":
    
    fake_on = rospy.get_param("~fake_on", 0)
    
    try:
        if fake_on == 1:
            
            repeat = rospy.get_param("~repeat_rate", 15)
            pub_thread = PublishThread(repeat)
            pub_thread.update(True)
            print('Faking drone takeoff')
        elif fake_on == 0:
            print('Relying on Gazebo for drone takeoff')
    except Exception as e:
        print(e)