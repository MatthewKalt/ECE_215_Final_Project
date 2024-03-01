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
        #==============================Grab bread and place it respective position==========================================
        BreadQuat = tfutil.convert_quat(env.sim.data.get_body_xquat('Bread_main'))
        BreadQuat = tfutil.quat_multiply(BreadQuat, tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward.
        desiredPose = (env.sim.data.get_body_xpos('Bread_main'), BreadQuat ) 

        jointAngles = inverseKinematics(DesiredPose_in_U=desiredPose, env=env)
        Your_gripper_EEF_pose = getGripperEEFPose(env, jointAngles)
        env.render()


        DesiredBreadQuat = tfutil.convert_quat(env.sim.data.get_body_xquat('VisualBread_main'))
        DesiredBreadQuat = tfutil.quat_multiply(DesiredBreadQuat, tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward.
        desiredPose = (env.sim.data.get_body_xpos('VisualBread_main'), DesiredBreadQuat ) 

        jointAngles = inverseKinematics(DesiredPose_in_U=desiredPose, env=env)
        Your_gripper_EEF_pose = getGripperEEFPose(env, jointAngles)
        env.render()


        #==============================Grab milk and place it respective position==========================================
        MilkQuat = tfutil.convert_quat(env.sim.data.get_body_xquat('Milk_main'))
        MilkQuat = tfutil.quat_multiply(MilkQuat, tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward.
        desiredPose = (env.sim.data.get_body_xpos('Milk_main'), MilkQuat ) 

        jointAngles = inverseKinematics(DesiredPose_in_U=desiredPose, env=env)
        Your_gripper_EEF_pose = getGripperEEFPose(env, jointAngles)

        env.render()

        DesiredMilkQuat = tfutil.convert_quat(env.sim.data.get_body_xquat('VisualMilk_main'))
        DesiredMilkQuat = tfutil.quat_multiply(DesiredMilkQuat, tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward.
        desiredPose = (env.sim.data.get_body_xpos('VisualMilk_main'),  DesiredMilkQuat ) 

        jointAngles = inverseKinematics(DesiredPose_in_U=desiredPose, env=env)
        Your_gripper_EEF_pose = getGripperEEFPose(env, jointAngles)
        env.render()


        #==============================Grab can and place it respective position==========================================
        CanQuat = tfutil.convert_quat(env.sim.data.get_body_xquat('Can_main'))
        CanQuat = tfutil.quat_multiply(CanQuat, tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward.
        desiredPose = (env.sim.data.get_body_xpos('Can_main'), CanQuat ) 
        jointAngles = inverseKinematics(DesiredPose_in_U=desiredPose, env=env)
        Your_gripper_EEF_pose = getGripperEEFPose(env, jointAngles)
        env.render()

        DesiredCanQuat = tfutil.convert_quat(env.sim.data.get_body_xquat('VisualCan_main'))
        DesiredCanQuat = tfutil.quat_multiply(DesiredCanQuat, tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward
        desiredPose = (env.sim.data.get_body_xpos('VisualCan_main'),  DesiredCanQuat ) 
        jointAngles = inverseKinematics(DesiredPose_in_U=desiredPose, env=env)
        Your_gripper_EEF_pose = getGripperEEFPose(env, jointAngles)
               
        env.render()
      
        #==============================Grab cereal and place it respective position==========================================
        CerealQuat = tfutil.convert_quat(env.sim.data.get_body_xquat('Cereal_main'))
        CerealQuat = tfutil.quat_multiply(CerealQuat, tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward.
        desiredPose = (env.sim.data.get_body_xpos('Cereal_main'), CerealQuat ) 
        jointAngles = inverseKinematics(DesiredPose_in_U=desiredPose, env=env)
        Your_gripper_EEF_pose = getGripperEEFPose(env, jointAngles)
        env.render()

        DesiredCerealQuat = tfutil.convert_quat(env.sim.data.get_body_xquat('VisualCereal_main'))
        DesiredCerealQuat = tfutil.quat_multiply(DesiredCerealQuat, tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward
        desiredPose = (env.sim.data.get_body_xpos('VisualCereal_main'),  DesiredCerealQuat ) 
        jointAngles = inverseKinematics(DesiredPose_in_U=desiredPose, env=env)
        Your_gripper_EEF_pose = getGripperEEFPose(env, jointAngles)
        env.render()


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
        control_freq=1,  # control should happen fast enough so that simulation looks smoother
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
    

