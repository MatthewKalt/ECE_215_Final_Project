import xml.etree.ElementTree as ET
import os
import numpy as np
import robosuite.utils.transform_utils as tfutil
import copy
import time


def inverseKinematics(RunNum,DesiredPose_in_U = (np.zeros(3,), np.array([0., 0., 0., 1.])), env = []):

    robotBasePose = (env.robots[0].base_pos, env.robots[0].base_ori) 
    # if RunNum == 0:
    initialJointAngles= env.robots[0]._joint_positions
     
    jointAngles = initialJointAngles.copy()
    
    #============= Your code here =============
    Desired_Pos = DesiredPose_in_U[0]
    Desired_Quat = DesiredPose_in_U[1]
    Desired_Error = .025
    EEF_Pos = getGripperEEFPose(env,jointAngles)[0]
    EEF_Quat = getGripperEEFPose(env,jointAngles)[1]

    fullcurrent=np.array([[0,0,0]])
    fulldesired=np.array([[0,0,0]])
    fulltime = np.array([0])
    PoseErrorCumSum=np.zeros(6)
    dThetaprev=np.zeros(7)

    NumSteps = 1500
    StepCount = 0
    Jacobian_Calc = getJacobian(env)
    Jacobian_Inv = np.linalg.pinv(Jacobian_Calc)
# -------------------------------------------------------------------------
    #iterate until position is reached
    Time = time.time()
    TimePrev=Time
    Iterm = 0
    while StepCount < NumSteps:
        Time =time.time()
        EEF_Pos = getGripperEEFPose(env,jointAngles)[0]
        EEF_Quat = getGripperEEFPose(env,jointAngles)[1]
     
        PoseError = CalcPoseError(Desired_Quat,Desired_Pos,EEF_Quat,EEF_Pos)
        
        
        
        PoseErrorCumSum+=PoseError
        
        if np.linalg.norm(PoseError) < Desired_Error:
            break
        
        
        fullcurrent=np.append(fullcurrent,np.array([EEF_Pos]),axis=0)
        fulldesired=np.append(fulldesired,np.array([Desired_Pos]),axis=0)
    
        Jacobian_Calc = getJacobian(env)
        Jacobian_Inv = np.linalg.pinv(Jacobian_Calc)

        dTheta = np.matmul(Jacobian_Inv,PoseError)
        dThetaCumSum = np.matmul(Jacobian_Inv, PoseErrorCumSum)
        dThetaCumSum += dTheta

       
        #time.sleep(.01)
        boost=np.array([1,1,1,1,1,1,10,1])
        if RunNum >= 1:
            fname="toshelf"
            # Kp=.4
            # Ki=0.0
            # Kd= 0.01
            Kp= 2
            Ki=0.0001
            Kd= 3
            Pterm=dTheta*Kp
            #Iterm=dThetaCumSum*Ki
            Iterm = Iterm + Ki*dTheta*(Time-TimePrev)
            Dterm=(dTheta-dThetaprev)*Kd/(Time-TimePrev)
            NewdTheta=Pterm+Iterm+Dterm
            NewdTheta = np.append(NewdTheta,0.020833)
            NewdTheta = NewdTheta*boost
            action = NewdTheta
            
            obs, reward, done, info = env.step(action)  # take action in the environment
           
        else:
            fname="tocube"
            Kp= 2
            Ki= 0.0001
            Kd= 3
            Pterm=dTheta*Kp
            Iterm=dThetaCumSum*Ki
            Dterm=(dTheta-dThetaprev)*Kd/(Time-TimePrev)
            NewdTheta=Pterm+Iterm+Dterm
            NewdTheta = np.append(NewdTheta,-1)
            NewdTheta=NewdTheta*boost
            action = NewdTheta# sample random action
            obs, reward, done, info = env.step(action)  # take action in the environment
            
        
        env.render()
        StepCount += 1
        dThetaprev=dTheta
        TimePrev = Time

  
    #==========================================
    #getGripperEEFPose(env, initialJointAngles) # Brings the robot to the initial joint angle.
    env.render()
    
    print("SUCCESS")
    
    np.savetxt(fname+"current.txt",fullcurrent)
    np.savetxt(fname+"desired.txt",fulldesired)

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
   
   
    # env.robots[0].set_robot_joint_positions(setJointAngles)
    gripper_EEF_pose = (env.robots[0].sim.data.get_body_xpos('gripper0_eef'), tfutil.convert_quat(env.robots[0].sim.data.get_body_xquat('gripper0_eef')))     
    return gripper_EEF_pose # Outputs the position and quaternion (x,y,z,w) of the EEF pose in Universial Frame{0}.
    

def getJacobian(env): # This function returns the jacobian of current configurations
    jacp = env.robots[0].sim.data.get_body_jacp('gripper0_eef').reshape((3, -1))[:,env.robots[0]._ref_joint_vel_indexes]
    jacr = env.robots[0].sim.data.get_body_jacr('gripper0_eef').reshape((3, -1))[:,env.robots[0]._ref_joint_vel_indexes]    
    jacobianMat_gripperEEF = np.concatenate((jacp, jacr),axis=0)
    return jacobianMat_gripperEEF #Outputs the Jacobian expressed in {0}

