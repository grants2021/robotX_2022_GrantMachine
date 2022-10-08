#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 28 11:54:55 2022

@author: grant
"""

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


class Node():
    def __init__(self):
        self.takeoff_pub = rospy.Publisher('/drone/takeoff_tag',Bool,queue_size=1)

    def publish(self,input_vel_cmds):
        self.takeoff_pub.publish(True)

            

if __name__ == "__main__":
    global timer, takeofftag
    rospy.init_node('fake_takeoff', anonymous=False)
    node = Node()
    turn_on = rospy.get_param("~turn_on",0)
    if turn_on == 1:
        node.publish()
        print('Faking drone takeoff')
    elif turn_on == 0:
        print('Relying on Gazebo for drone takeoff')