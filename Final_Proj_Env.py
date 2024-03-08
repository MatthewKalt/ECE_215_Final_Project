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

def MoveUp(env, ItemString):
        BodyMat = env.sim.data.get_body_xmat(ItemString)
        BodyPos = env.sim.data.get_body_xpos(ItemString)
        MoveMat = np.zeros([4,4])
        LiftPos = BodyPos + [0.,.0,0.4]
        MoveMat[:3,:3] = BodyMat
        MoveMat[:3,3] = LiftPos 
        
        return LiftPos, tfutil.mat2quat(MoveMat)

      
def Moveright(env, ItemString):
        BodyMat = env.sim.data.get_body_xmat(ItemString)
        BodyPos = env.sim.data.get_body_xpos(ItemString)
        MoveMat = np.zeros([4,4])
        LiftPos = BodyPos + [0.,0.3,0.]
        MoveMat[:3,:3] = BodyMat
        MoveMat[:3,3] = LiftPos 
        
        return LiftPos, tfutil.mat2quat(MoveMat)

def Turnright(env, ItemString):
        BodyMat = env.sim.data.get_body_xmat(ItemString)
        BodyPos = env.sim.data.get_body_xpos(ItemString)
        MoveMat = np.zeros([4,4])
        LiftPos = BodyPos + [0.,0.0,0.]
        MoveMat[:3,:3] = BodyMat
        MoveMat[:3,3] = LiftPos 
        
        return LiftPos, tfutil.mat2quat(MoveMat)

def GoToItemDest(Item):
    BodyPos = env.sim.data.get_body_xpos(Item)
   


     
      
def GetObjects(env = any):

    action = [0,0,0,0,0,0,0,0]
    obs, reward, done, info = env.step(action)  # take action in the environment
    #ItemList = FPL.GetItemList(env,obs)

    #for key,value in ItemList.items():
    for i in range(1):
        ItemString = 'cube_main'

#GOING TO ITEM FROM START========================================================
        ItemQuat = tfutil.convert_quat(env.sim.data.get_body_xquat(ItemString))
        ItemQuat = tfutil.quat_multiply(ItemQuat,tfutil.axisangle2quat([0.,np.pi,0.0]))# tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward.
        DesiredPose = (env.sim.data.get_body_xpos(ItemString),ItemQuat) 
        jointAngles = FPL.inverseKinematics(0,.025,DesiredPose_in_U=DesiredPose, env=env)
        Your_gripper_EEF_pose = FPL.getGripperEEFPose(env, jointAngles)
#============================================================================================        
        FPM.CloseGripper(env)

        DesiredOrientation = [0.,np.pi,0.0]
        FPM.MoveZ(env,ItemString,0.3,DesiredOrientation)

        FPM.MoveY(env,ItemString,0.3,DesiredOrientation)

        # LiftPos, LiftQuat = Turnright(env,ItemString)
        LiftQuat = [0.0,-0.707,-0.707,0.0]
        FPM.Rotate(env, ItemString, LiftQuat)
        # DesiredPose = (LiftPos,LiftQuat) 
        # jointAngles = FPL.inverseKinematics(1,.025,DesiredPose_in_U=DesiredPose, env=env)
      
 
     

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

        GetObjects(env)
        env.render()

        input('Hit Enter to test new pose')
    # env.close()
    
