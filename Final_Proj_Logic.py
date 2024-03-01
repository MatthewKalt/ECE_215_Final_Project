import xml.etree.ElementTree as ET
import os
import numpy as np
import robosuite.utils.transform_utils as tfutil
import copy



def inverseKinematics(DesiredPose_in_U = (np.zeros(3,), np.array([0., 0., 0., 1.])), env = []):
    # These two OPTIONAL helper functions will actually set the angles and get you the gripper endeffector pose and jacobian.    
    #  "getGripperEEFPose" is actually moving the robot in the simulation but it does not render it. This works as a forward kinematics function. If you want to see the new robot pose, add: env.render()
    # "getJacobian(env)" returns the Jacobian computed for the gripper end-effector which is different from what you get in HW3. 

    #getGripperEEFPose(env, setJointAngles)
    #getJacobian(env)

    # We will bring the robot back to original pose at the end of "inverseKinematics" function, because it is inteded to compute the joint angles, not execute the joint angles.
    # But it is not required for you to implement it.

    # Tuple of position and orientation (quat) of the base frame expressed in world frame
    robotBasePose = (env.robots[0].base_pos, env.robots[0].base_ori) 
    initialJointAngles= env.robots[0]._joint_positions
    jointAngles = initialJointAngles.copy()
    
    #============= Your code here =============
    Desired_Pos = DesiredPose_in_U[0]
    Desired_Quat = DesiredPose_in_U[1]
    Desired_Error = .001
    EEF_Pos = getGripperEEFPose(env,jointAngles)[0]
    EEF_Quat = getGripperEEFPose(env,jointAngles)[1]

    NumSteps = 100
    StepCount = 0
    # 1) Get pose error
    #      calc position error 
    #      calc rotation error
    #      put together and send off
    #
    # 2) if norm(error) < desired error we win
    #
    # 3) get jacobian
    #
    # 4) dTheta = dot(pinv(jacob),error)
    #
    # 5) update angles

    Jacobian_Calc = getJacobian(env)
    Jacobian_Inv = np.linalg.pinv(Jacobian_Calc)
# -------------------------------------------------------------------------
    #iterate until position is reached
    while StepCount < NumSteps:
        EEF_Pos = getGripperEEFPose(env,jointAngles)[0]
        EEF_Quat = getGripperEEFPose(env,jointAngles)[1]
        #Calculate and store error in position :: abs(desired-actual)
        PoseError = CalcPoseError(Desired_Quat,Desired_Pos,EEF_Quat,EEF_Pos)

        if np.linalg.norm(PoseError) < Desired_Error:
            break

        Jacobian_Calc = getJacobian(env)
        Jacobian_Inv = np.linalg.pinv(Jacobian_Calc)

        dTheta = np.matmul(Jacobian_Inv,PoseError)

        jointAngles += dTheta
        getGripperEEFPose(env,jointAngles) # Brings the robot to the initial joint angle.
        env.render()
        StepCount += 1

  
    #==========================================
    getGripperEEFPose(env, initialJointAngles) # Brings the robot to the initial joint angle.
    env.render()
    return jointAngles


def CalcPoseError(DesiredQuat,DesiredPos,ActualQuat,ActualPos):
    DesiredRotMat = tfutil.quat2mat(DesiredQuat)
    ActualRotMat = tfutil.quat2mat(ActualQuat)
    
    DesiredPose = tfutil.make_pose(DesiredPos,DesiredRotMat)
    ActualPose = tfutil.make_pose(ActualPos,ActualRotMat)

    PoseError = tfutil.get_pose_error(DesiredPose,ActualPose)

    return PoseError

#=========== Not a HW problem below ==========

def getGripperEEFPose(env, setJointAngles): # This function works as a forward Kinematics

    env.robots[0].set_robot_joint_positions(setJointAngles)
    gripper_EEF_pose = (env.robots[0].sim.data.get_body_xpos('gripper0_eef'), tfutil.convert_quat(env.robots[0].sim.data.get_body_xquat('gripper0_eef')))     
    return gripper_EEF_pose # Outputs the position and quaternion (x,y,z,w) of the EEF pose in Universial Frame{0}.
    

def getJacobian(env): # This function returns the jacobian of current configurations
    jacp = env.robots[0].sim.data.get_body_jacp('gripper0_eef').reshape((3, -1))[:,env.robots[0]._ref_joint_vel_indexes]
    jacr = env.robots[0].sim.data.get_body_jacr('gripper0_eef').reshape((3, -1))[:,env.robots[0]._ref_joint_vel_indexes]    
    jacobianMat_gripperEEF = np.concatenate((jacp, jacr),axis=0)
    return jacobianMat_gripperEEF #Outputs the Jacobian expressed in {0}

