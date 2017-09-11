#Backup of the main_old code move these contents into the main_old inorder to get back working 



import pybullet as p
import matplotlib.pyplot as plt
import time

# ##Function to rotate right wheel in forward or rev direction
# def wheel_right_rot(vel):
#     #vel is velocity of wheel    
#     p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, vel)
#     p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, vel)
#     p.stepSimulation()
#     p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, 0.0)
#     p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, 0.0)
#     return
#     
# ##Function to rotate left wheel in forward or rev direction
# def wheel_left_rot(vel):
#     #vel is velocity of wheel    
#     p.setJointMotorControl2(boxId, 3, p.VELOCITY_CONTROL, vel)
#     p.setJointMotorControl2(boxId, 5, p.VELOCITY_CONTROL, vel)
#     p.stepSimulation()
#     p.setJointMotorControl2(boxId, 3, p.VELOCITY_CONTROL, 0.0)
#     p.setJointMotorControl2(boxId, 5, p.VELOCITY_CONTROL, 0.0)
#     return

#connect to GUI or DIRECT
physicsClient = p.connect(p.GUI)

#Set gravity
p.setGravity(0,0,-10)

#Import ground plane
planeId = p.loadURDF("plane.urdf")

#Import obstacle box
obsboxStartPos = [5,0,0.3]
obsboxStartOrientation = p.getQuaternionFromEuler([0,0,0])
obsId = p.loadURDF("box/urdf/box.urdf",obsboxStartPos, obsboxStartOrientation)

#Import HMMR
cubeStartPos = [0,0,0.14]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("HMMR/urdf/HMMR.urdf",cubeStartPos, cubeStartOrientation)

# pos = []
# for i in range(100):
#     p.stepSimulation()
#     cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
#     print(cubePos[2])
#     pos.append(cubePos[2])
#     time.sleep(0.01)

for i in range(p.getNumJoints(boxId)):
    print(p.getJointInfo(boxId,i))  
  

#move forward 
for i in range(235):     
    ang = -i*0.1
    p.setJointMotorControl2(boxId, 2, p.POSITION_CONTROL, ang)
    p.setJointMotorControl2(boxId, 3, p.POSITION_CONTROL, ang)
    p.setJointMotorControl2(boxId, 4, p.POSITION_CONTROL, ang)
    p.setJointMotorControl2(boxId, 5, p.POSITION_CONTROL, ang)    
    p.stepSimulation()
    time.sleep(0.01)   
    
#raise motion platform
for i in range(200):
    p.setJointMotorControl2(boxId, 0, p.POSITION_CONTROL, 0.0157*i)
    p.stepSimulation()
    time.sleep(0.01)  

for i in range(150):     
    ang = -(235+i)*0.1
    p.setJointMotorControl2(boxId, 1, p.POSITION_CONTROL, -0.0157*i)
    p.setJointMotorControl2(boxId, 2, p.POSITION_CONTROL, ang)
    p.setJointMotorControl2(boxId, 3, p.POSITION_CONTROL, ang)
    p.setJointMotorControl2(boxId, 4, p.POSITION_CONTROL, ang)
    p.setJointMotorControl2(boxId, 5, p.POSITION_CONTROL, ang)    
    p.stepSimulation()
    time.sleep(0.01)   

for i in range(50):
    p.setJointMotorControl2(boxId, 0, p.POSITION_CONTROL, 0.0157*(i+200))
    p.stepSimulation()
    time.sleep(0.01)  
    
for i in range(235):     
    ang = -(235+150+i)*0.1
    p.setJointMotorControl2(boxId, 2, p.POSITION_CONTROL, ang)
    p.setJointMotorControl2(boxId, 3, p.POSITION_CONTROL, ang)
    p.setJointMotorControl2(boxId, 4, p.POSITION_CONTROL, ang)
    p.setJointMotorControl2(boxId, 5, p.POSITION_CONTROL, ang)    
    p.stepSimulation()
    time.sleep(0.01)  
    
for i in range(700):
    p.stepSimulation()
    time.sleep(0.01)  

p.disconnect()
# plt.plot(pos)
# plt.ylabel('height')
# plt.xlabel('time')
# plt.show()

#Next stop get contact information.
#But is that actually the next step
#Reduce height of the platfrom to climb better
#Add small wheels multiple ones on the model to climb better
