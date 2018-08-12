# -*- coding: utf-8 -*-
"""
Created on Fri Sep 29 17:12:01 2017

@author: jose
"""

import numpy as np
import sympy as sp
import sys
import time
import networkx as nx
import warnings
import matplotlib.pyplot as plt
sp.init_printing()

class Transformation:
    def obtainTransform(self,start,end):
        if start is end:
            return np.identity(4) #Necessary otherwise and error appears on the shortes_path function

        chain = nx.shortest_path(self.G, start , end)[:-1]
        #print chain
        transformations = list()
        for j in chain:
            transformations.append(HomogeneousTransformation(self.G.node[j]["DH"]))
    
        B = np.identity(4)
        
        for z in transformations:
            B = z.H * B

        return B, chain

class HomogeneousTransformation:
    def __init__(self,DH): #DH = [d,a,theta,alpha]
        assert len(DH) == 4        
        dh_ = list()

        type_ = type(DH[0])
        
        for i in range(len(DH)):
            if not isinstance(DH[i],type_):
                sys.exit("Denavit Hartenberg parameters must be from same type per joint")

            if isinstance(DH[i],str): # 
                dh_.append(sp.Symbol(DH[i]))#string
            else:
                if i is 2 or i is 3:
                    dh_.append(DH[i]* np.pi / 180) #number
                else:
                    dh_.append(DH[i])

        if type_ is str:
            self.H = sp.Matrix(sp.ones(4,4))
            c = sp.cos
            s = sp.sin
        else:
            self.H = np.matrix(np.ones((4,4)))
            c = np.cos
            s = np.sin 
        
        #First Row
        self.H[0,0] = c(dh_[2])
        self.H[0,1] = -s(dh_[2]) * c(dh_[3])
        self.H[0,2] = s(dh_[2]) *s(dh_[3])
        self.H[0,3] =  c(dh_[1]) * dh_[1] 

        #Second Row
        self.H[1,0] = s(dh_[2])
        self.H[1,1] = c(dh_[2]) * c(dh_[3])
        self.H[1,2] = -c(dh_[2]) *s(dh_[3])
        self.H[1,3] =  s(dh_[2]) * dh_[1]
       
        #THird Row
        self.H[2,0] = 0
        self.H[2,1] = s(dh_[3])
        self.H[2,2] = c(dh_[3])
        self.H[2,3] = dh_[0]
        #Forth Row
        self.H[3,0] = 0
        self.H[3,1] = 0
        self.H[3,2] = 0
        self.H[3,3] = 1
        #print self.H

    def printTransformation(self):
        print (sp.pretty(self.H))

class Chain(Transformation):

    def __init__(self, DH, joints_names):
        """
        Joint i of name joints_names i will be automatically 
        linked with the joint i+1
        using the row i from the DH parameters list
        """

        self.G = nx.Graph()
        row = 0
        for i in range(len(joints_names)):
            #Add Node with name joint_names[i] and DH parameters of row i
            self.G.add_node(joints_names[i], DH = [])
            self.G.node[joints_names[i]]["DH"] = DH[row]
            # Additional information of Node provided start is itself
            # Since a homogeneous tranformation matrix usually
            # discribes two frams transforamtion starting from the frame i
            self.G.node[joints_names[i]]["start"] = i
            row= row + 1
            
            if i is not len(joints_names)-1:
                #Additionally information to the nodes is added 
                #Joint i+1 is going to be set as end of the homogeneous
                # transformation
                self.G.node[joints_names[i]]["end"] = joints_names[i+1]
                #Edge from node i to node i+1
                self.G.add_edge(joints_names[i],joints_names[i+1])
        
        #print self.G.nodes()

class HeadChain:
    def __init__(self, sim = False):
        self.DH = list()
        
        if sim:
            self.DH.append(['d_0','a_1','th_0','al_0'])
            self.DH.append(['d_1','a_1','th_1','al_1'])
            self.DH.append(['d_2','a_2','th_2','al_1'])
            self.DH.append(['0','0','0','0'])
 
        else:
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])

        self.joints_names = ["HeadEndEffector", "HeadYaw", "HeadPitch", "base_link"]
        self.chain = Chain(self.DH,self.joints_names)

class ArmChain:

    def __init__(self, prefix = "L", sim = False):

        self.DH = list()
        
        if sim: #sympy
            if prefix is 'L':
                self.DH.append(['d_a0','a_a1','th_a0','al_a0'])
                self.DH.append(['d_a1','a_a1','th_a1','al_a1'])
                self.DH.append(['d_a2','a_a2','th_a2','al_a2'])
                self.DH.append(['d_a3','a_a3','th_a3','al_a3'])
                self.DH.append(['d_a4','a_a4','th_a4','al_a4'])
                self.DH.append(['d_a5','a_a5','th_a5','al_a5'])
                self.DH.append(['d_a6','a_a6','th_a6','al_a6'])
                self.DH.append(['d_a7','a_a7','th_a7','al_a7'])
            else:
                self.DH.append(['d_ar0','a_ar1','th_ar0','al_ar0'])
                self.DH.append(['d_ar1','a_ar1','th_ar1','al_ar1'])
                self.DH.append(['d_ar2','a_ar2','th_ar2','al_ar2'])
                self.DH.append(['d_ar3','a_ar3','th_ar3','al_ar3'])
                self.DH.append(['d_ar4','a_ar4','th_ar4','al_ar4'])
                self.DH.append(['d_ar5','a_ar5','th_ar5','al_ar5'])
                self.DH.append(['d_ar6','a_ar6','th_ar6','al_ar6'])
                self.DH.append(['d_ar7','a_ar7','th_ar7','al_ar7'])

        elif prefix is 'L':
            #DH LEFT ARM
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
        else:
            #DH RIGHT ARM
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])
            self.DH.append([0,0,0,0])

        self.joints_names = ["base_link", prefix + "ShoulderPitch3", 
                             prefix +"ShoulderRoll3", prefix + "ElbowYaw3", 
                             prefix + "ElbowRoll3", prefix + "WristYaw3",
                             "NAO_" + prefix + "ThumbBase",
                             "NAO_" + prefix + "LFingerBase",
                             "NAO_" + prefix + "RFingerBase"]
        self.chain = Chain(self.DH,self.joints_names)

class LegChain:

    def __init__(self, prefix = "L", sim = False):
        self.DH = list()
        
        if sim: #sympy
            if prefix is 'L':
                self.DH.append(['d_l0','a_l1','th_l0','al_l0'])
                self.DH.append(['d_l1','a_l1','th_l1','al_l1'])
                self.DH.append(['d_l2','a_l2','th_l2','al_l2'])
                self.DH.append(['d_l3','a_l3','th_l3','al_l3'])
                self.DH.append(['d_l4','a_l4','th_l4','al_l4'])
                self.DH.append(['d_l5','a_l5','th_l5','al_l5'])
                self.DH.append(['d_l6','a_l6','th_l6','al_l6'])
                self.DH.append(['d_l7','a_l7','th_l7','al_l7'])
            else:
                self.DH.append(['d_lr0','a_lr1','th_lr0','al_lr0'])
                self.DH.append(['d_lr1','a_lr1','th_lr1','al_lr1'])
                self.DH.append(['d_lr2','a_lr2','th_ar2','al_lr2'])
                self.DH.append(['d_lr3','a_lr3','th_ar3','al_lr3'])
                self.DH.append(['d_lr4','a_lr4','th_ar4','al_lr4'])
                self.DH.append(['d_lr5','a_lr5','th_ar5','al_lr5'])
                self.DH.append(['d_lr6','a_lr6','th_ar6','al_lr6'])
                self.DH.append(['d_lr7','a_lr7','th_ar7','al_lr7'])
 
        elif prefix is 'L':
            #DH LEFT LEG d a th al
            self.DH.append([0,0,90,135])
            self.DH.append([93,0,-90,180])
            self.DH.append([0,0,0,90])
            self.DH.append([0,0,-135,-90])
            self.DH.append([0,93,0,0])
            self.DH.append([0,93,0,0])
            self.DH.append([0,0,0,90])
            self.DH.append([0,33.5,0,0])
            self.DH.append([52,0,0,90])
            self.DH.append([0,0,0,0])

        else:
            #DH RIGHT LEG
            self.DH.append([0,0,90,45])
            self.DH.append([93,0,90,180])
            self.DH.append([0,0,0,-90])
            self.DH.append([0,0,135,-90])
            self.DH.append([0,93,0,0])
            self.DH.append([0,93,0,0])
            self.DH.append([0,0,0,90])
            self.DH.append([0,33.5,0,0])
            self.DH.append([52,0,0,90])
            self.DH.append([0,0,0,0])

        self.joints_names = [ "base_link", prefix + "HipYawPitch3", 
                             prefix + "HipRoll3", prefix + "HipPitch3", 
                             prefix + "KneePitch3", prefix + "AnklePitch3", 
                             prefix +"AnkleRoll3", prefix + "End_Efector"]
        self.chain = Chain(self.DH,self.joints_names)

class NAO(Transformation):
    def __init__(self):
        
        #swarnings.warn("If SYMPY transformation are used for all joints program get stuck")
        
        self.leftarm = ArmChain("L")
        self.rightarm = ArmChain("R")
        self.leftleg = LegChain("L")
        self.rightleg = LegChain("R")
        self.head = HeadChain()

        self.G = nx.Graph()
        self.G.add_nodes_from(self.head.chain.G.nodes(data=True))
        self.G.add_edges_from(self.head.chain.G.edges())
        self.G.add_nodes_from(self.leftarm.chain.G.nodes(data=True))
        self.G.add_edges_from(self.leftarm.chain.G.edges())
        self.G.add_nodes_from(self.rightarm.chain.G.nodes(data=True))
        self.G.add_edges_from(self.rightarm.chain.G.edges())
        self.G.add_nodes_from(self.leftleg.chain.G.nodes(data=True))
        self.G.add_edges_from(self.leftleg.chain.G.edges())
        self.G.add_nodes_from(self.rightleg.chain.G.nodes(data=True))
        self.G.add_edges_from(self.rightleg.chain.G.edges())
        #print self.G.nodes()
    
    def plotGraph(self):
        plt.figure()
        #nx.draw_networkx(self.G,pos=nx.grap(self.G))

        nx.draw_networkx(self.G,pos=nx.spring_layout(self.G))
        #nx.draw_networkx(self.G, pos = nx.spectral_layout(self.G,scale=1,weight='end'))
        plt.show()
    
    def inverseKinematics(self, part, current_poses, goal_pose, goal_frame):
        if part is "head":
            return [goal_pose[1],goal_pose[2]]
        if part is "leftarm":
            l1 , l2, l3 , l4 = 0
            q0, q1, q2, q3, q4, q5, q6 = 0
            np.pi - np.arccos()
            return [q0,q1,q2,q3,q4,q5,q6]
        return False
            
        