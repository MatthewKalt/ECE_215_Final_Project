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




def GetObjects(env = any):
       
    ItemList = ['cube_main']#,"VisualBread","Milk","VisualMilk","Can","VisualCan","Cereal","VisualCereal"]
    for i in range(len(ItemList)):
        print(type(env.robots[0]))
        ItemString = ItemList[i]
        ItemQuat = tfutil.convert_quat(env.sim.data.get_body_xquat(ItemString))
        ItemQuat = tfutil.quat_multiply(ItemQuat, tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward.
        DesiredPose = (env.sim.data.get_body_xpos(ItemString),ItemQuat) 

        jointAngles = inverseKinematics(i,DesiredPose_in_U=DesiredPose, env=env)
        Your_gripper_EEF_pose = getGripperEEFPose(env, jointAngles)
        print(help(env.robots[0].control))
        # print(env.robots[0].gripper_type)
        # print(env.robots[0].has_gripper)
        # print(env.robots[0].gripper.format_action([1]))
        # env.robots[0].grip_action(env.robots[0].gripper,[1])

        # for i in range(100):
        #     env.render()
        #     #print(env.robots[0].gripper.format_action([1]))

        #     time.sleep(.1)
        
     

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
        control_freq= .1 , # control should happen fast enough so that simulation looks smoother
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
    
