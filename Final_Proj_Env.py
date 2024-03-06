"""
This script shows you how to select gripper for an environment.
This is controlled by gripper_type keyword argument.
"""
import numpy as np
import time

import robosuite as suite
from robosuite import ALL_GRIPPERS
import robosuite.utils.transform_utils as tfutil

from Final_Proj_Logic import *

def MoveUp(env, ItemString):
        BodyMat = env.sim.data.get_body_xmat(ItemString)
        BodyPos = env.sim.data.get_body_xpos(ItemString)
        MoveMat = np.zeros([4,4])
        LiftPos = BodyPos + [0.,0.3,0.4]
        MoveMat[:3,:3] = BodyMat
        MoveMat[:3,3] = LiftPos 
        
        return LiftPos, tfutil.mat2quat(MoveMat)

def GetObjects(env = any):
       
    ItemList = ['cube_main']#,"VisualBread","Milk","VisualMilk","Can","VisualCan","Cereal","VisualCereal"]
    for i in range(len(ItemList)):
           
        ItemString = ItemList[i]
        #print(env.sim.data.get_body_xquat(ItemString))
        ItemQuat = tfutil.convert_quat(env.sim.data.get_body_xquat(ItemString))
        ItemQuat = tfutil.quat_multiply(ItemQuat,[0,1,0,0])# tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward.
        DesiredPose = (env.sim.data.get_body_xpos(ItemString),ItemQuat) 

        jointAngles = inverseKinematics(0,DesiredPose_in_U=DesiredPose, env=env)
        Your_gripper_EEF_pose = getGripperEEFPose(env, jointAngles)
        
        for j in range(20):
                   # print((env.robots[0].dof))
                    action = [0,0,0,0,0,0,0,.020833] # sample random action
                    obs, reward, done, info = env.step(action)  # take action in the environment
                    env.render()  # render on display]

        LiftPos, LiftQuat = MoveUp(env,ItemString)
       # print(tfutil.axisangle2quat([0.,np.pi,0.0]))

        LiftQuat = [-0.0, -0.707,  -.707, -0.0000]

        DesiredPose = (LiftPos,LiftQuat) 

        jointAngles = inverseKinematics(1,DesiredPose_in_U=DesiredPose, env=env)
        #Your_gripper_EEF_pose = getGripperEEFPose(env, jointAngles)

    
 
     

if __name__ == "__main__":
    
    gripper = 'PandaGripper'

    # Notify user which gripper we're currently using
    print("Using gripper {}...".format(gripper))
    
    np.set_printoptions(precision=4, floatmode='fixed', suppress=True)

    # create environment with selected grippers
    env = suite.make(
        "Lift",
        robots="Panda",
        gripper_types=gripper,
        
        has_renderer=True,  # make sure we can render to the screen
        has_offscreen_renderer=False,  # not needed since not using pixel obs
        use_camera_obs=False,  # do not use pixel observations
        control_freq= 25 , # control should happen fast enough so that simulation looks smoother
        camera_names="frontview",
        
    )
    
    
    while(True):
        # Reset the env
        env.reset()

 
        print('===============================================')
        # print('Desired Pose', desiredPose)        
        # print('Your inverse kinematics result ', Your_gripper_EEF_pose)
        print('===============================================')    
        GetObjects(env)
        env.render()

        input('Hit Enter to test new pose')
    # env.close()
    
