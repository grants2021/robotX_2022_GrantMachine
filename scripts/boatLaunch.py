#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 16:17:44 2022

@author: grant
"""
#import wamv_dk_functions as dkw
import roslaunch
import sys
import rospy

#node = roslaunch.core.Node('robotx_2022','wamv_basic_commands.launch')
#rospy.init_node('fuckme',anonymous=True)
#uuid = roslaunch.rlutil.get_or_generate_uuid(None,False)
#roslaunch.configure_logging(uuid)


#if False:
namespace = "/wamv"
#node = dkw.Node()
#node.constant([1,1,0])
#node.subscriber('burst',[1,1,0],20)
remaplist = ["left_thrust_angle",  namespace+"/thrusters/right_thrust_angle",\
             "right_thrust_angle", namespace+"/thrusters/right_thrust_angle",\
                 "left_thrust_cmd", namespace+"/thrusters/left_thrust_cmd",\
                     "right_thrust_cmd", namespace+"/thrusters/right_thrust_cmd"]
#paramlist = ["input_right_cmd:=1","input_left_cmd:=1","input_angle:=0","iter_num:=0"]
paramlist = ["input_right_cmd:=1","input_left_cmd:=1","input_angle:=0","iter_num:=0"]
paramstr = ''
for param in paramlist:
    paramstr = paramstr+param
    sys.argv.append(param)
wamv_dk_node = roslaunch.core.Node("robotx_2022","wamv_dk_functions.py","wamv_dk_functions")
                #remap_args=remaplist)#, args=paramstr)
#    <remap from= to="/$(arg namespace)/thrusters/left_thrust_angle"/>
#    <remap from= to="/$(arg namespace)/thrusters/right_thrust_angle"/>
#    <remap from= to="/$(arg namespace)/>
#    <remap from= to="/$(arg namespace)/thrusters/right_thrust_cmd"/>
#pkg= type= name= output="screen"

#launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/grant/vrx_ws/src/adept-vrx/robotX_2022/launch/wamv_basic_commands.launch'])
#node = roslaunch.core.Node("robotx_2022","wamv_dk_functions.py","wamv_dk_functions")
node = roslaunch.core.Node("robotx_2022","wamv_basic_commands")
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()
process = launch.launch(node)