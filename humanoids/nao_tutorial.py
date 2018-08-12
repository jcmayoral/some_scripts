# -*- coding: utf-8 -*-
"""
Created on Tue Sep 26 16:10:20 2017

@author: jose
"""

import vrep
import time
import sys
import numpy as np
import matplotlib.pyplot as plt

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')   
else:
    print ('Failed connecting to remote API server')
    sys.exit("could not connect")


head_joints_name = ["HeadYaw", "HeadPitch"]
l_leg_joints_name = ["LHipYawPitch3", "LHipRoll3", "LHipPitch3", "LKneePitch3", "LAnklePitch3","LAnkleRoll3"]
r_leg_joints_name = ["RHipYawPitch3", "RHipRoll3", "RHipPitch3", "RKneePitch3", "RAnklePitch3","RAnkleRoll3"]
l_arm_joints_name = ["LShoulderPitch3","LShoulderRoll3","LElbowYaw3","LElbowRoll3","LWristYaw3","NAO_LThumbBase","NAO_LLFingerBase","NAO_LRFingerBase"]
r_arm_joints_name = ["RShoulderPitch3","RShoulderRoll3","RElbowYaw3","RElbowRoll3","RWristYaw3","NAO_RThumbBase","NAO_RLFingerBase","NAO_RRFingerBase"]

joint_names = [head_joints_name, l_leg_joints_name, r_leg_joints_name, l_arm_joints_name, r_arm_joints_name]

handlers_ = []

for x in joint_names:
    for a in x:
        errorCode,handle=vrep.simxGetObjectHandle(clientID, a ,vrep.simx_opmode_oneshot_wait)
        #print errorCode
        handlers_.append(handle)
#errorCode = vrep.simxSetJointTargetVelocity(clientID, head_yaw_handle, 1, vrep.simx_opmode_oneshot_wait)
#errorCode = vrep.simxSetJointPosition(clientID,head_yaw_handle,0.10,vrep.simx_opmode_oneshot_wait)

for h in handlers_:
    #vrep.simxSetJointTargetVelocity(clientID, h , 1, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetPosition(clientID,h,-5, vrep.simx_opmode_oneshot_wait)
#
#errorCode = vrep.simxSetJointPosition(clientID,head_yaw_handle,10,vrep.simx_opmode_oneshot_wait)
#print errorCode

 # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Now close the connection to V-REP:
vrep.simxFinish(clientID)
