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
# baseDir = '/home/yashima/'
baseDir = '/home/docker/'
timeStep = 1./240.
captureImageShape = (400,400)
boardCenterPos = [0,0.2,-0.67]
modelName = 'GQCNN-2.0'
modelDir = '/home/yashima/gqcnn/models/'
resetDebugVisualizerCameraParameter = dict(cameraDistance=0.8, cameraYaw=38, cameraPitch=-22, cameraTargetPosition=boardCenterPos)
# resetDebugVisualizerCameraParameter = dict(cameraDistance=0.6, cameraYaw=0, cameraPitch=-89, cameraTargetPosition=boardCenterPos)
# resetDebugVisualizerCameraParameter = dict(cameraDistance=0.6, cameraYaw=89, cameraPitch=-1, cameraTargetPosition=boardCenterPos)
# resetDebugVisualizerCameraParameter = dict(cameraDistance=0.6, cameraYaw=0, cameraPitch=-10, cameraTargetPosition=boardCenterPos)
projectionFarVal = 0.6
projectionNearVal = boardCenterPos[1] - 0.006
projectionFov = 40
viewMatrix = p.computeViewMatrix(cameraEyePosition=[boardCenterPos[0], boardCenterPos[1]+projectionFarVal, boardCenterPos[2]+1e-6], cameraTargetPosition=boardCenterPos, cameraUpVector=[0, 1, 0])
# policyMode = 'manual' #'plan'or'manual'
policyMode = 'plan' #'plan'or'manual'

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
panda = panda_sim.PandaSimAuto(p, boardCenterPos)
panda.control_dt = timeStep

### Capture camera image
projectionMatrix = p.computeProjectionMatrixFOV(fov=projectionFov, aspect=1.*captureImageShape[0]/captureImageShape[1], nearVal=projectionNearVal, farVal=projectionFarVal)
_,_,rgbaImage,depthImage,segmentImage = p.getCameraImage(*captureImageShape, viewMatrix=viewMatrix, projectionMatrix=projectionMatrix)
depthImage = np.expand_dims(depthImage, -1)
depthImage = projectionFarVal * projectionNearVal / (projectionFarVal - (projectionFarVal - projectionNearVal) * depthImage)
segmentImage = np.array(segmentImage*255, np.uint8)
bgrImage = rgbaImage[:,:,[2,1,0]]
cv2.imwrite('./data/images/color.png', bgrImage)
np.save('./data/images/depth.npy', depthImage)
cv2.imwrite('./data/images/segmask.png', segmentImage)

### Convert 'px' to 'm'
def pxxy2mxy(posPxXY):
    #Todo: modify to analitical calculation
    posPxXonBoard = posPxXY[0] - 14
    posPxYonBoard = posPxXY[1] - 14
    dxDdpx = 0.4/(383-14)
    dyDdpy = 0.4/(383-14)
    posX = boardCenterPos[0] - 0.2 + dxDdpx * posPxXonBoard
    posY = boardCenterPos[2] - 0.2 + dyDdpy * posPxYonBoard
    return [posX, posY]

#################################################################
# Generate policy
#################################################################
#ToDo (policty1: random policy for collecting training samples)
#if policyMode == 'random':
#	policy = gen_random_policy(depthImage, segmentImage)
if policyMode == 'plan':
    graspAction = planner.plan(baseDir,modelName,'./data/images/depth.npy','./data/images/segmask.png','./data/intr/camera.intr')
    posXY = pxxy2mxy(graspAction.center)
    posZ = boardCenterPos[1] + projectionFarVal - graspAction.depth
    radZ = graspAction.angle
    policy = [posXY, posZ, radZ]
elif policyMode == 'manual':
	# posXY = pxxy2mxy([85,83])
	posXY = pxxy2mxy([200,315])
	posZ = boardCenterPos[1] + 0.02
	radZ = 0.
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
