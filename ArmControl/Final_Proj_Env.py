"""
This script shows you how to select gripper for an environment.
This is controlled by gripper_type keyword argument.
"""
import numpy as np
import time

import robosuite as suite
from robosuite import ALL_GRIPPERS
import robosuite.utils.transform_utils as tfutil

import Final_Proj_Logic as FPL
import Final_Proj_Movement as FPM




# 
    
      
def GetObjects(env = any):

    action = [0,0,0,0,0,0,0,0]
    obs, reward, done, info = env.step(action)  # take action in the environment
    ItemList = FPL.GetItemList(env,obs)
    #key = 'cube_main'
    print(obs)
    for key,value in ItemList.items():
    #for i in range(1):
        ItemString = key
        #print(env.sim.data.get_body_xpos("robot0_base"))
       # print(ItemString)
#GOING TO ITEM FROM START========================================================
        ItemQuat = tfutil.convert_quat(env.sim.data.get_body_xquat(ItemString))
        ItemQuat = tfutil.quat_multiply(ItemQuat,tfutil.axisangle2quat([0.,np.pi,0.0]))# tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward.
        DesiredPose = (env.sim.data.get_body_xpos(ItemString),ItemQuat) 
        DesiredPoseSlightAbove=(env.sim.data.get_body_xpos(ItemString)+[0.0,0.0,0.05],ItemQuat) 
        
        jointAngles = FPL.inverseKinematics(0,.025,DesiredPose_in_U=DesiredPoseSlightAbove, env=env)
        print("Finish align_on_top")
        jointAngles = FPL.inverseKinematics(0,.015,DesiredPose_in_U=DesiredPose, env=env)
        print("Finish align_coincedent")
        # Your_gripper_EEF_pose = FPL.getGripperEEFPose(env, jointAngles)
#============================================================================================        
        FPM.CloseGripper(env)
        print("grab")
        DesiredOrientation = [0.,np.pi,0.0]
        
        FPM.MoveZ(env,ItemString,0.2,DesiredOrientation)
        print("Finish MoveZ")

        

        FPM.MoveXY(env,"Visual"+ItemString,DesiredOrientation)
        print("Finish MoveY")
       # LiftQuat = [0.0,-0.707,-0.707,0.0]

        FPM.OpenGripper(env)
        print("Finish Release")
        #FPM.Rotate(env, ItemString, LiftQuat)=============================        
        
        
      
 
     
if __name__ == "__main__":
    
    gripper = 'PandaGripper'

    # Notify user which gripper we're currently using
    print("Using gripper {}...".format(gripper))
    
    np.set_printoptions(precision=4, floatmode='fixed', suppress=True)

    # create environment with selected grippers
    env = suite.make(
        "PickPlace",
        robots="Panda",
        gripper_types=gripper,
        
        has_renderer=True,  # make sure we can render to the screen
        has_offscreen_renderer=False,  # not needed since not using pixel obs
        use_camera_obs=False,  # do not use pixel observations

        control_freq= 25 , # control should happen fast enough so that simulation looks smoother
        horizon=5000,
        camera_names="frontview",

        
    )
    
    
    while(True):
        # Reset the env
        env.reset()
        print(suite.models.objects)

        GetObjects(env)
        env.render()

        input('Hit Enter to test new pose')
    # env.close()
    
    