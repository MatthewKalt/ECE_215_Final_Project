import xml.etree.ElementTree as ET
import os
import numpy as np
import robosuite.utils.transform_utils as tfutil
import copy
import time

import Final_Proj_Logic as FPL

STEP_SIZE = 0.20
ERROR = 0.1

def CloseGripper(env):
    for j in range(20):
        action = [0,0,0,0,0,0,0,.020833] # sample random action
        obs, reward, done, info = env.step(action)  # take action in the environment
        env.render()  # render on display]

def MoveX(env,Item,Dist,Ori):
    BodyMat = env.sim.data.get_body_xmat(Item)
    BodyPos = env.sim.data.get_body_xpos(Item)
    MoveMat = np.zeros([4,4])
    CurrGoal = 0.0

    while CurrGoal <= Dist:
        LiftPos = BodyPos + [CurrGoal,0.0,0.0]
        MoveMat[:3,:3] = BodyMat
        MoveMat[:3,3] = LiftPos 
        LiftQuat = tfutil.mat2quat(MoveMat)
        LiftQuat = tfutil.quat_multiply(tfutil.convert_quat(env.sim.data.get_body_xquat(Item)),tfutil.axisangle2quat(Ori))
        DesiredPose = (LiftPos,LiftQuat) 

        jointAngles = FPL.inverseKinematics(1,ERROR,DesiredPose_in_U=DesiredPose, env=env)
        CurrGoal += STEP_SIZE

def MoveY(env,Item,Dist,Ori):
    BodyMat = env.sim.data.get_body_xmat(Item)
    BodyPos = env.sim.data.get_body_xpos(Item)
    MoveMat = np.zeros([4,4])
    CurrGoal = 0.0

    while CurrGoal <= Dist:
        LiftPos = BodyPos + [0.0,CurrGoal,0.0]
        MoveMat[:3,:3] = BodyMat
        MoveMat[:3,3] = LiftPos 
        LiftQuat = tfutil.mat2quat(MoveMat)
        LiftQuat = tfutil.quat_multiply(tfutil.convert_quat(env.sim.data.get_body_xquat(Item)),tfutil.axisangle2quat(Ori))
        DesiredPose = (LiftPos,LiftQuat) 

        jointAngles = FPL.inverseKinematics(1,ERROR,DesiredPose_in_U=DesiredPose, env=env)
        CurrGoal += STEP_SIZE


def MoveZ(env,Item,Dist,Ori):
 
    BodyMat = env.sim.data.get_body_xmat(Item)
    BodyPos = env.sim.data.get_body_xpos(Item)
    MoveMat = np.zeros([4,4])
    CurrGoal = 0.0

    while CurrGoal <= Dist:
        LiftPos = BodyPos + [0.0,0.0,CurrGoal]
        MoveMat[:3,:3] = BodyMat
        MoveMat[:3,3] = LiftPos 
        LiftQuat = tfutil.mat2quat(MoveMat)
        LiftQuat = tfutil.quat_multiply(tfutil.convert_quat(env.sim.data.get_body_xquat(Item)),tfutil.axisangle2quat(Ori))
        DesiredPose = (LiftPos,LiftQuat) 

        jointAngles = FPL.inverseKinematics(1,ERROR,DesiredPose_in_U=DesiredPose, env=env)
        CurrGoal += STEP_SIZE
