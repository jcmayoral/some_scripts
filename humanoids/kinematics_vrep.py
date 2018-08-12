# -*- coding: utf-8 -*-
"""
Created on Mon Oct  2 10:28:23 2017

@author: jose
"""

import vrep
from kinematics import nao
from vrep_connection import robot_connection

myrobot = robot_connection.VREP_Robot(nao.NAO)
__handlers = dict()
__current_pose = dict()

for x in myrobot.joint_names:
    for a in x:
        errorCode,handle=vrep.simxGetObjectHandle(myrobot.clientID, a ,vrep.simx_opmode_oneshot_wait)
        __handlers[a] = handle


#dk_, chain = myrobot.robot.obtainTransform( "base_link","LEnd_Efector") #LEG
dk_, chain = myrobot.robot.obtainTransform( "base_link","LWristYaw3")
#dk_, chain = myrobot.robot.obtainTransform( "base_link","HeadEndEffector") #HEAD
#myrobot.robot.plotGraph()
print chain

for h in __handlers.items():
    joint_value = vrep.simxGetJointPosition(myrobot.clientID,__handlers[h[0]],vrep.simx_opmode_oneshot_wait)
    __current_pose[h[0]] = joint_value[1]
    
#print __current_pose
    
start_poses = {}

#ik = myrobot.robot.inverseKinematics("head", __current_pose, [0,-0.5,-0.5], "HeadEndEffector")
ik = myrobot.robot.inverseKinematics("leftarm", __current_pose, [0,-0.3,-0.3], "HeadEndEffector")

print ik

j = 0
if ik is False:
    print "IK Failure"
    quit()    

for i in chain: #base_link does not exists
    if i is "base_link":
        continue
    value = ik[j] #instead of this this should be inverse matrix
    vrep.simxSetJointTargetPosition(myrobot.clientID,__handlers[i],value, vrep.simx_opmode_oneshot_wait)
    j = j+1

myrobot.closeConnection()