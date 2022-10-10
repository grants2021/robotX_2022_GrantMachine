#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 22:01:12 2022

@author: grant
"""

import dict_dk_functions as dkf 
gazeboOrQGC = 1 #1 for gazebo 0 for QGroundControl
qrttag = True #True=on False=off


#launchName = 'wamv_qr_empty.launch'
#launchName = 'iris_double_world.launch'
#launchName = 'drone_wamv_qr_empty.launch'
#launchName = 'platformed_drone_wamv_qr_empty.launch'
#launchName = 'visualworld_platformed_drone_wamv_qr.launch'
launchName = 'grass_replenish_platform.launch'

dkf.launcher(gazeboOrQGC, qrttag, launchName)