import xml.etree.ElementTree as ET
import os
import numpy as np
import robosuite.utils.transform_utils as tfutil
import copy
import time

import Final_Proj_Logic as FPL


#define allowable tolerance to via point before continuing
DIVIDE = 1
ERROR = 0.02

'''
brings gripper to a close through velocity control
ARGS: env
'''
def CloseGripper(env):
    for j in range(20):
        action = [0,0,0,0,0,0,0,.020833] # sample random action
        obs, reward, done, info = env.step(action)  # take action in the environment
        env.render()  # render on display]

'''
Moves end effector through space in the X direction
ARGS: env - Item[string] - Dist[float] - Ori[axis angles]
'''
def MoveX(env,Item,Dist,Ori):
    #Grabbing items body data and initializing function variables 
    BodyMat = env.sim.data.get_body_xmat(Item)
    BodyPos = env.sim.data.get_body_xpos(Item)
    MoveMat = np.zeros([4,4])
    CurrGoal = 0.0
    StepSize = Dist/DIVIDE
    #increment through via checkpoints until final destination is reached
    while CurrGoal <= Dist:
        LiftPos = BodyPos + [CurrGoal,0.0,0.0]
        MoveMat[:3,:3] = BodyMat
        MoveMat[:3,3] = LiftPos 
        LiftQuat = tfutil.mat2quat(MoveMat)
        LiftQuat = tfutil.quat_multiply(tfutil.convert_quat(env.sim.data.get_body_xquat(Item)),tfutil.axisangle2quat(Ori))
        DesiredPose = (LiftPos,LiftQuat) 
        jointAngles = FPL.inverseKinematics(1,ERROR,DesiredPose_in_U=DesiredPose, env=env)
        CurrGoal += StepSize

'''
Moves end effector through space in the Z direction
ARGS: env - Item[string] - Dist[float] - Ori[axis angles]
'''
def MoveY(env,Item,Dist,Ori):
    #Grabbing items body data and initializing function variables
    BodyMat = env.sim.data.get_body_xmat(Item)
    BodyPos = env.sim.data.get_body_xpos(Item)
    MoveMat = np.zeros([4,4])
    CurrGoal = 0.0
    StepSize = Dist/DIVIDE
    #increment through via checkpoints until final destination is reached
    while CurrGoal <= Dist:
        LiftPos = BodyPos + [0.0,CurrGoal,0.0]
        MoveMat[:3,:3] = BodyMat
        MoveMat[:3,3] = LiftPos 
        LiftQuat = tfutil.mat2quat(MoveMat)
        LiftQuat = tfutil.quat_multiply(tfutil.convert_quat(env.sim.data.get_body_xquat(Item)),tfutil.axisangle2quat(Ori))
        DesiredPose = (LiftPos,LiftQuat) 
        jointAngles = FPL.inverseKinematics(1,ERROR,DesiredPose_in_U=DesiredPose, env=env)
        CurrGoal += StepSize
'''
Moves end effector through space in the Z direction
ARGS: env - Item[string] - Dist[float] - Ori[axis angles]
'''
def MoveZ(env,Item,Dist,Ori):
    #Grabbing items body data and initializing function variables
    BodyMat = env.sim.data.get_body_xmat(Item)
    BodyPos = env.sim.data.get_body_xpos(Item)
    MoveMat = np.zeros([4,4])
    CurrGoal = 0.0
    StepSize = Dist/DIVIDE
    #increment through via checkpoints until final destination is reached
    while CurrGoal <= Dist:
        LiftPos = BodyPos + [0.0,0.0,CurrGoal]
        MoveMat[:3,:3] = BodyMat
        MoveMat[:3,3] = LiftPos 
        LiftQuat = tfutil.mat2quat(MoveMat)
        LiftQuat = tfutil.quat_multiply(tfutil.convert_quat(env.sim.data.get_body_xquat(Item)),tfutil.axisangle2quat(Ori))
        DesiredPose = (LiftPos,LiftQuat) 
        jointAngles = FPL.inverseKinematics(1,ERROR,DesiredPose_in_U=DesiredPose, env=env)
        CurrGoal += StepSize

def Rotate(env,Item,Quat):
    # LiftPos, LiftQuat = Turnright(env,ItemString)
    # LiftQuat = [0.0,-0.707,-0.707,0.0]
    BodyMat = env.sim.data.get_body_xmat(Item)
    BodyPos = env.sim.data.get_body_xpos(Item)
    MoveMat = np.zeros([4,4])
    LiftPos = BodyPos + [0.,0.3,0.]
    MoveMat[:3,:3] = BodyMat
    MoveMat[:3,3] = LiftPos 
        

    DesiredPose = (LiftPos,Quat) 
    jointAngles = FPL.inverseKinematics(1,.025,DesiredPose_in_U=DesiredPose, env=env)
      
 