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


# MOTORS
l_error_code, left_motor = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
r_error_code, right_motor = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)

#SPEED
l_vel_error_code = vrep.simxSetJointTargetVelocity(clientID, left_motor, 0.40, vrep.simx_opmode_streaming)
r_vel_error_code = vrep.simxSetJointTargetVelocity(clientID, right_motor,-0.40, vrep.simx_opmode_streaming)
 
#ULTRASONIC SENSOR
s1_error_code, sensor_1 = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor1', vrep.simx_opmode_oneshot_wait)
s1_error_code, s1_state, s1_point, s1_obj_handler, s1_surface_vector = vrep.simxReadProximitySensor(clientID, sensor_1, vrep.simx_opmode_streaming)

while not s1_state:
    s1_error_code, s1_state, s1_point, s1_obj_handler, s1_surface_vector = vrep.simxReadProximitySensor(clientID, sensor_1, vrep.simx_opmode_buffer)

#camera
cam_error_code, camera_handle = vrep.simxGetObjectHandle(clientID,'cam1', vrep.simx_opmode_oneshot_wait)
cam_error_code, resolution, image = vrep.simxGetVisionSensorImage(clientID,camera_handle,0,vrep.simx_opmode_oneshot_wait)

cam_error_code, resolution, image = vrep.simxGetVisionSensorImage(clientID,camera_handle,0,vrep.simx_opmode_blocking)
# No idea why the next line does not work
#cam_error_code, resolution, image = vrep.simxGetVisionSensorImage(clientID,camera_handle,0,vrep.simx_opmode_buffer)
im = np.array(image, dtype=np.uint8)
im.resize([resolution[0],resolution[1],3])
plt.imshow(im, origin='lower')


 # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Now close the connection to V-REP:
vrep.simxFinish(clientID)
