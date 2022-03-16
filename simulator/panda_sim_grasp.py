'''
Modify from https://github.com/csingh27/Bin-Picking-Simulation-using-Pybullet/Demo\ Simulation/panda_sim_grasp.py
'''

import time
from turtle import distance
import numpy as np
import math
import random

CONTINUE = 'continue'
END = 'end'


useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
#restposes for null space
jointPositions=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 0, 0.02, 0.02]
rp = jointPositions

class PandaSim(object):
  def __init__(self, bullet_client, board_center):
    self.bullet_client = bullet_client
    self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)
    
    #print("offset=",offset)
    flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    self.board_center = board_center
    self.legos=[]
    
    # self.bullet_client.loadURDF("tray/traybox.urdf", board_center, [-0.5, -0.5, -0.5, 0.5], flags=flags)
    self.bullet_client.loadURDF("../data/urdf/plane.urdf", board_center[:2]+[board_center[2]], [-0.5, -0.5, -0.5, 0.5], flags=flags)
    # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([board_center[0]+0.05, board_center[1]+0.034, board_center[2]-0.05]), flags=flags))
    # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([board_center[0]+0.05, board_center[1]+0.034, board_center[2]-0.07]), flags=flags))
    self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([board_center[0]+random.uniform(-0.1,0.1), board_center[1]+0.034, board_center[2]+random.uniform(-0.1,0.1)]), flags=flags))
    # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([-0.06, 0.034, -0.65]), flags=flags))
    # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([board_center[0]-0.125, board_center[1]+0.034, board_center[2]-0.125]), flags=flags))
    # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([board_center[0]+0.2, board_center[1]+0.034, board_center[2]+0.2]), flags=flags))
    # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([board_center[0]+0.2, board_center[1]+0.034, board_center[2]-0.2]), flags=flags))
    # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([board_center[0]-0.2, board_center[1]+0.034, board_center[2]+0.2]), flags=flags))
    # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([board_center[0]-0.2, board_center[1]+0.034, board_center[2]-0.2]), flags=flags))
    # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.05, 0.034, -0.65]), flags=flags))
    # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.05, 0.034, -0.47]), flags=flags))
    # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([-0.12, 0.034, -0.58]), flags=flags))
    self.bullet_client.changeVisualShape(self.legos[0],-1,rgbaColor=[1,0,0,1])
    orn=[-0.707107, 0.0, 0.0, 0.707107]#p.getQuaternionFromEuler([-math.pi/2,math.pi/2,0])
    self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", np.array([0.1,0,0]), orn, useFixedBase=True, flags=flags)
    self.bullet_client.setJointMotorControl2(self.panda, 0, self.bullet_client.POSITION_CONTROL, -0.6, force=5 * 240.)
    index = 0
    self.state = 0
    self.control_dt = 1./240.
    self.finger_target = 0
    self.gripper_default_height = board_center[1] +  0.15
    self.gripper_rad = 0
    #create a constraint to keep the fingers centered
    c = self.bullet_client.createConstraint(self.panda,
                       9,
                       self.panda,
                       10,
                       jointType=self.bullet_client.JOINT_GEAR,
                       jointAxis=[1, 0, 0],
                       parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
    self.bullet_client.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)
 
    for j in range(self.bullet_client.getNumJoints(self.panda)):
      self.bullet_client.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
      info = self.bullet_client.getJointInfo(self.panda, j)
      #print("info=",info)
      jointName = info[1]
      jointType = info[2]
      if (jointType == self.bullet_client.JOINT_PRISMATIC):
        
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
      if (jointType == self.bullet_client.JOINT_REVOLUTE):
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
    self.t = 0.
    
    while self.t < 2:
      self.bullet_client.stepSimulation()
      self.t += self.control_dt
    
  def set_policy(self, policy):
    xy_pos, depth, rad = policy
    self.target_gripper_pos = [xy_pos[0], depth, xy_pos[1]]
    self.target_gripper_rad = rad
  
  def is_grasp_succes(self):
    #Todo
    #return 0 or 1
    pass
    
  def reset(self):
    pass

  def update_state(self):
    pass
  
  def fixed_target_value(self, target_value, init_value):
    time_ratio = 0.8
    return min(self.state_t,self.state_dt*time_ratio)/(self.state_dt*time_ratio)*target_value + max(self.state_dt*time_ratio-self.state_t,0)/(self.state_dt*time_ratio)*init_value
  
  def step(self):
    status = self.update_state()
    if status == END:
      legoid = self.legos[0]
      legopos = self.bullet_client.getBasePositionAndOrientation(legoid)
      print('legopos:',legopos)
      print('Is success:',legopos[0][1]>=self.board_center[1]+0.08)
      return status
    
    t = self.t
    self.t += self.control_dt
    if self.state in [0,1,4,5]:
      # pos = [0.2 * math.sin(1.5 * t), self.gripper_default_height, -0.6 + 0.1 * math.cos(1.5 * t)]
      if self.state == 0:
        pos = self.target_gripper_pos
        pos = [pos[0], self.gripper_default_height, pos[2]]
        self.prev_pos = pos
        self.gripper_rad = self.target_gripper_rad
      if self.state == 1:
        pos = self.target_gripper_pos
        self.prev_pos = pos
      if self.state == 4:
        gripper_height = self.gripper_default_height
        pos = [self.prev_pos[0], gripper_height, self.prev_pos[2]]
        self.prev_pos = pos
      if self.state == 5:
        pos = self.prev_pos
        diffX = pos[0]
        diffZ = pos[2] + 0.6
        self.prev_pos = [self.prev_pos[0] - diffX*0.1, self.prev_pos[1], self.prev_pos[2]-diffZ*0.1]
      	
      orn = self.bullet_client.getQuaternionFromEuler([math.pi/2.,0.,0.])
      # jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaEndEffectorIndex, pos, orn, maxNumIterations=20)
      jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaEndEffectorIndex, pos, orn, ll, ul, jr, rp, maxNumIterations=200)
      jointPoses = list(jointPoses)
      jointPoses[6] = self.gripper_rad
      self.bullet_client.submitProfileTiming()
      for i in range(pandaNumDofs):
        init_pos = self.initJointPoses[i]
        target_pos = jointPoses[i]
        target_pos = self.fixed_target_value(target_pos, init_pos)
        self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, target_pos,force=5 * 240.)
        
    #target for fingers
    elif self.state in [2,3]:
      if self.state==6:
          self.finger_target = 0.01
      if self.state==5:
        self.finger_target = 0.04
      for i in [9,10]:
        init_pos = self.initJointPoses[i]
        target_pos = self.finger_target
        target_pos = self.fixed_target_value(target_pos, init_pos)
        self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, target_pos,force= 10)
        
    self.bullet_client.stepSimulation()
    
    return status

class PandaSimAuto(PandaSim):
  def __init__(self, bullet_client, board_center):
    PandaSim.__init__(self, bullet_client, board_center)
    self.state_t = 0
    self.state_dt = 0
    self.cur_state = 0
    self.states=[0,1,2,3,4,5]
    self.state_durations=[1.5,4,1.5,4,1.5,1.5,1.5]
  
  def update_state(self):
    if self.state_t == 0:
      self.initJointPoses = []
      for i in range(pandaNumDofs+5):
        joint_pos = self.bullet_client.getJointState(self.panda, i)[0]
        self.initJointPoses.append(joint_pos)
        
    self.state_t += self.control_dt
    self.state_dt = self.state_durations[self.cur_state]
    if self.state_t > self.state_dt:
      self.cur_state += 1
      if self.cur_state>=len(self.states):
        return END
      self.state_t = 0
      self.state=self.states[self.cur_state]
      #print("self.state=",self.state)
    return CONTINUE
