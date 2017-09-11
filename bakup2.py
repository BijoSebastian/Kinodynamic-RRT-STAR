import pybullet as p
import matplotlib.pyplot as plt
import time
import numpy as np

global pose 
global goal
global boxId

####################The PyBullet setup###########################
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
pose = [0.0, 0.0, 0.0, 0.0, 0.0]

#Identifiers
# for i in range(p.getNumJoints(boxId)):
#     print(p.getJointInfo(boxId,i))  
# Revlt_jnt1ID = 0;
# Revlt_jnt2ID = 1;
# FRwhl_jntID = 2;
# FLwhl_jntID = 3;
# BRwhl_jntID = 4;
# BLwhl_jntID = 5;
   
##########################Controllers for Robot###############
def pub_vel(v, w):
    #Function to publish the desired linear
    #and angular velocity of the robot
    
    #Parameters
    R = 0.7 #radius of wheels
    L  = 1 #total distance between wheels  

    #Compute desired vel_r, vel_l needed to ensure w
    Vr = ((2.0*v) + (w*L))/(2*R)
    Vl = ((2.0*v) - (w*L))/(2*R)
                
    #Set theta1 
    p.setJointMotorControl2(boxId, 0, p.POSITION_CONTROL, targetPosition = 0.0, force = 750)
     
    #Set theta2  
    p.setJointMotorControl2(boxId, 1, p.POSITION_CONTROL, targetPosition = 0.0, force = 750)
    
    print ("Vr :", Vr, "Vl :", Vl)                         
    #Set Vr and Vl
    for i in range(10):
        p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity = -Vr, force = 500)
        p.setJointMotorControl2(boxId, 3, p.VELOCITY_CONTROL, targetVelocity = -Vl, force = 500)
        p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, targetVelocity = -Vr, force = 500)
        p.setJointMotorControl2(boxId, 5, p.VELOCITY_CONTROL, targetVelocity = -Vl, force = 500)
        p.stepSimulation()
        time.sleep(0.01)          
    return

def pose_update():
    #Update the pose information of the robot
    #pose is x, y positontion and theta orientation of base
    #theta1 and theta2 angles of the two links
     
    global pose
    
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    cubeOrn = p.getEulerFromQuaternion(cubeOrn)   
    pose = [cubePos[0], cubePos[1], cubeOrn[2], 0.0, 0.0]
    print (pose)    
    return

def at_goal():
    #Event which checks if we have reached the goal(within threshold)     
    
    global goal
    global pose

    #The threshold distance 
    distThresh = 0.1#mm
    
    #get the goal location
    xd = goal[0]
    yd = goal[1]
    
    #get the current robot location
    xa = pose[0]
    ya = pose[1]
    
    #check if we have reached goal point
    d = np.sqrt(pow((xd - xa),2) + pow((yd - ya),2))
    
    if d <= distThresh:
        print ("Reached goal")
        return True
    else:
        return False

def gtg():
    #The Go to goal controller
    
    global goal
    global pose
    
    #Controller parameters
    Kp = 1.5
       
    #get the goal location
    xd = goal[0]
    yd = goal[1]
    
    #get the current robot location
    xa = pose[0]
    ya = pose[1]
    thetaa = pose[2]
    
    #determine how far to rotate to face the goal point
    #PS. ALL ANGLES ARE IN RADIANS
    dt = (np.arctan2((yd - ya), (xd -xa))) - thetaa
    #restrict angle to (-pi,pi)
    dt = ((dt + np.pi)%(2.0*np.pi)) - np.pi
    dt = ((dt*180.0)/np.pi)
        
    #control input for angular velocity
    W = (Kp*dt) 
  
    #find distance to goal
    d = np.sqrt(pow((xd - xa),2) + pow((yd - ya),2))
    
    #control input for linear velocity
    if d > 0.05 :
        V = 10.0
    else :
        V = 0.0
        
    print ("V :", V, "W :", W)    
    #request robot to execute velocity
    pub_vel(V,W)
    return

#################################MAIN#############
if __name__ == '__main__':
    #The main_old funtion
    
    global goal
    goal  =  [-5.0, -5.0, 0.0, 0.0, 0.0]

    while not at_goal():        
        gtg()
        #p.stepSimulation()        
        #time.sleep(5.0)  
        pose_update()
    
    pub_vel(0.0, 0.0)
    
    for i in range(700):
        p.stepSimulation()
        time.sleep(0.01)
        
    p.disconnect()
      