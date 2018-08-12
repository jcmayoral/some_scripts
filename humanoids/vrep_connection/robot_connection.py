import vrep
import sys

class VREP_Robot:
    
    def __init__(self, robot, robot_type = "Humanoid"):
        vrep.simxFinish(-1) # just in case, close all opened connections
        self.clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
        if self.clientID!=-1:
            print ('Connected to remote API server')   
        else:
            print ('Failed connecting to remote API server')
            sys.exit("could not connect")
            
        if robot_type is "Humanoid":
            self.initHumanoidRobot(robot)
        else:
            print ("No valid robot type")
            
    def initHumanoidRobot(self, robot_class):
        self.robot = robot_class()
        self.joint_names = [self.robot.head.joints_names, 
                            self.robot.leftleg.joints_names,
                            self.robot.rightleg.joints_names, 
                            self.robot.leftarm.joints_names, 
                            self.robot.rightarm.joints_names]
    
    def closeConnection(self):
        # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(self.clientID)
        # Now close the connection to V-REP:
        vrep.simxFinish(self.clientID)
