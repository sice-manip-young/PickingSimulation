import pybullet as p
import pybullet_data as pd
import time
from copy import deepcopy
import numpy as np
import cv2
from simulator import panda_sim_grasp as panda_sim
import planner


#################################################################
# Parameter setup
#################################################################
timeStep = 1./240.
captureImageShape = (400,400)
modelName = 'GQCNN-2.0'
modelDir = '/home/yashima/gqcnn/models/'
resetDebugVisualizerCameraParameter = dict(cameraDistance=1.3, cameraYaw=38, cameraPitch=-22, cameraTargetPosition=[0.35,-0.13,0])
viewMatrix = p.computeViewMatrix(cameraEyePosition=[0, 0.6, -0.5999], cameraTargetPosition=[0, 0, -0.6], cameraUpVector=[0, 1, 0])
projectionMatrix = p.computeProjectionMatrixFOV(fov=45, aspect=1.0, nearVal=0.01, farVal=1.6)
policyMode = 'manual' #'plan'or'manual'
# policyMode = 'plan' #'plan'or'manual'

#################################################################
# Pybullet init setup
#################################################################
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP,1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,1)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,1)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,1)
p.setPhysicsEngineParameter(maxNumCmdPer1ms=1000)
p.resetDebugVisualizerCamera(**resetDebugVisualizerCameraParameter)
p.setAdditionalSearchPath(pd.getDataPath())
p.setTimeStep(timeStep)
p.setGravity(0,-9.8,0)

#################################################################
# Panda simulator instance init setup
#################################################################
panda = panda_sim.PandaSimAuto(p)
panda.control_dt = timeStep

### Capture camera image
#Todo: modify image depth
_,_,rgbaImage,depthImage,segmentImage = p.getCameraImage(*captureImageShape, viewMatrix=viewMatrix, projectionMatrix=projectionMatrix)
depthImage = np.expand_dims(depthImage, -1)
segmentImage = np.array(segmentImage*255, np.uint8)
bgrImage = rgbaImage[:,:,[2,1,0]]
cv2.imwrite('./data/images/color.png', bgrImage)
np.save('./data/images/depth.npy', depthImage)
cv2.imwrite('./data/images/segmask.png', segmentImage)

### Convert 'px' to 'm'
def pxxy2mxy(posPxXY):
    #Todo: modify to analitical calculation
	lBottomX = 0.15384
	uTopY = -0.44654
	dxDpx = 0.00038461
	dyDpy = 0.00038461
	posX = lBottomX - dxDpx * posPxXY[0]
	posY = uTopY - dyDpy * posPxXY[1]
	return [posX, posY]

#################################################################
# Generate policy
#################################################################
#ToDo (policty1: random policy for collecting training samples)
#if policyMode == 'random':
#	policy = gen_random_policy(depthImage, segmentImage)
if policyMode == 'plan':
    graspAction = planner.plan(modelName,modelDir,'./data/images/depth.npy','./data/images/segmask.png','./data/intr/camera.intr')
    posXY = pxxy2mxy(graspAction.center)
    posZ = 1 - graspAction.depth
    radZ = graspAction.angle
    policy = [posXY, posZ, radZ]
elif policyMode == 'manual':
	posXY = pxxy2mxy([400,400])
	posZ = 0.03
	radZ = 0.7
	policy = [posXY, posZ, radZ]
panda.set_policy(policy)

#################################################################
# Run simulation
#################################################################
while True:
	panda.step()
	p.stepSimulation()
	time.sleep(timeStep)
	panda.bullet_client.submitProfileTiming()
