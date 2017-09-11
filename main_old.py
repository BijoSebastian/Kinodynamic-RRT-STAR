import pybullet as PBT
import matplotlib.pyplot as plt
import numpy as np
import random
import time
from math import sqrt

#RRT* parameters
Window_size = 20.0 #The operating window for each D.O.F
EPSILON = 0.3 #change
RADIUS = 15.0 #change
NUMNODES = 100 #increase

####################The PyBullet setup###########################
global boxId
#connect to GUI or DIRECT
PBT.connect(PBT.DIRECT)

#Set gravity
PBT.setGravity(0,0,-10)

#Import ground plane
PBT.loadURDF("plane.urdf", [12, 12, 0])

#Import obstacle boxes
obsboxStartPos = [5, 2, 0.0]
obsboxStartOrientation = PBT.getQuaternionFromEuler([0,0,0])
PBT.loadURDF("box/urdf/box.urdf",obsboxStartPos, obsboxStartOrientation)
obsboxStartPos = [5, 20, 0.0]
obsboxStartOrientation = PBT.getQuaternionFromEuler([0,0,0])
PBT.loadURDF("box/urdf/box.urdf",obsboxStartPos, obsboxStartOrientation)

#Check for feasibilty
#Import HMMR at desired pos
#cubeStartPos = [10, 12, 0.14]
#cubeStartOrientation = PBT.getQuaternionFromEuler([0.0, 0.0, 0])
#boxId = PBT.loadURDF("HMMR/urdf/HMMR.urdf",cubeStartPos, cubeStartOrientation)
#input("check")

##########################Controllers for Robot###############
global pose 
global goal

def pub_vel(v, w):    
    #Function to publish the desired linear
    #and angular velocity of the robot
    
    global boxId
    
    #Parameters
    R = 0.7 #radius of wheels
    L  = 1 #total distance between wheels  

    #Compute desired vel_r, vel_l needed to ensure w
    Vr = ((2.0*v) + (w*L))/(2*R)
    Vl = ((2.0*v) - (w*L))/(2*R)
                
    #Set theta1 
    PBT.setJointMotorControl2(boxId, 0, PBT.POSITION_CONTROL, targetPosition = 0.0, force = 750)
     
    #Set theta2  
    PBT.setJointMotorControl2(boxId, 1, PBT.POSITION_CONTROL, targetPosition = 0.0, force = 750)
    
    #print ("Vr :", Vr, "Vl :", Vl)                         
    #Set Vr and Vl
    for _ in range(10):
        PBT.setJointMotorControl2(boxId, 2, PBT.VELOCITY_CONTROL, targetVelocity = -Vr, force = 500)
        PBT.setJointMotorControl2(boxId, 3, PBT.VELOCITY_CONTROL, targetVelocity = -Vl, force = 500)
        PBT.setJointMotorControl2(boxId, 4, PBT.VELOCITY_CONTROL, targetVelocity = -Vr, force = 500)
        PBT.setJointMotorControl2(boxId, 5, PBT.VELOCITY_CONTROL, targetVelocity = -Vl, force = 500)
        PBT.stepSimulation()    
    return

def pose_update():
    #Update the pose information of the robot
    #pose is x, y positontion and theta orientation of base
    #theta1 and theta2 angles of the two links
     
    global pose
    
    cubePos, cubeOrn = PBT.getBasePositionAndOrientation(boxId)
    cubeOrn = PBT.getEulerFromQuaternion(cubeOrn)   
    pose = [cubePos[0], cubePos[1], cubeOrn[2], 0.0, 0.0]
    #print (pose)    
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
        #print ("Reached goal")
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
        
    #print ("V :", V, "W :", W)    
    #request robot to execute velocity
    pub_vel(V,W)
    return

####################RRT* setup##################################
def checkIntersect(p1, p2):
    # To check if intersections occur 
    #return true of there is collsion
    #or if robot is not able to rach goal within specified time  
    
    #print("in chk_intrsct")
    
    global goal
    global pose
    global boxId
    
    #Import HMMR at desired pos
    cubeStartPos = [p1.x, p1.y, 0.14]
    cubeStartOrientation = PBT.getQuaternionFromEuler([0.0, 0.0, p1.theta])
    boxId = PBT.loadURDF("HMMR/urdf/HMMR.urdf",cubeStartPos, cubeStartOrientation)
    
    #Identifiers
    # for i in range(PBT.getNumJoints(boxId)):
    #     print(PBT.getJointInfo(boxId,i))  
    # Revlt_jnt1ID = 0;
    # Revlt_jnt2ID = 1;
    # FRwhl_jntID = 2;
    # FLwhl_jntID = 3;
    # BRwhl_jntID = 4;
    # BLwhl_jntID = 5;    
    
    pose = [p1.x, p1.y, p1.theta, p1.theta1, p1.theta2]        
    goal = [p2.x, p2.y, p2.theta, p2.theta1, p2.theta2]
     
    #print ("pose", pose)
    #print ("goal", goal)
            
    temp_cost = 0    
    while  not at_goal() and temp_cost<100:          
        gtg()          
        pose_update()
        temp_cost += 1        
    
    pub_vel(0.0, 0.0)
    PBT.removeBody(boxId) 
    #print (temp_cost)
     
    if temp_cost< 100:
        return False
    else:
        return True
    
def dist(p1,p2):
    # Measure euclidean distance between nodes
    
    return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.theta - p2.theta)*(p1.theta - p2.theta) + (p1.theta1 - p2.theta1)*(p1.theta1 - p2.theta1) + (p1.theta2 - p2.theta2)*(p1.theta2 - p2.theta2))

def step_from_to(p1,p2):
    # Grow from current node to destination node as far as you can before collision. 
    
    #print("in step_from_to")
    max_dis= []
    
    temp = Node(p1.x, p1.y, p1.theta, p1.theta1, p1.theta2)    
    temp.x += (p2.x - p1.x)*EPSILON
    temp.y += (p2.y - p1.y)*EPSILON
    temp.theta += (p2.theta - p1.theta)*EPSILON
    temp.theta1 += (p2.theta1 - p1.theta1)*EPSILON
    temp.theta2 += (p2.theta2 - p1.theta2)*EPSILON
    #print ("temp", temp.x, temp.y, temp.theta)
    while not checkIntersect(p1, temp):
        max_dis = [temp.x, temp.y, temp.theta, temp.theta1, temp.theta2]
        temp.x += (p2.x - p1.x)*EPSILON
        temp.y += (p2.y - p1.y)*EPSILON
        temp.theta += (p2.theta - p1.theta)*EPSILON
        temp.theta1 += (p2.theta1 - p1.theta1)*EPSILON
        temp.theta2 += (p2.theta2 - p1.theta2)*EPSILON
        #print ("temp", temp.x, temp.y, temp.theta)
    
    if not max_dis:
        #print(p1.x, p1.y)
        #print(temp.x, temp.y)
        #print ("no")
        return p1
    else:
        return Node(max_dis[0], max_dis[1], max_dis[2], max_dis[3], max_dis[4])

def chooseParent(nn,newnode,nodes):
    # Choose the new parent among the neighbors within radius only of there is no collission provided distance from start is lesser
    
    for p in nodes:
        #print ("p.cost+dist", p.cost+ dist(p, newnode) )
        #print ("nn.cost+dist", nn.cost+dist(nn, newnode) )
        if p!= nn and dist(p, newnode) <RADIUS and p.cost+ dist(p, newnode) < nn.cost+dist(nn, newnode):
            if not checkIntersect(p, newnode):
                nn = p
    newnode.cost = nn.cost + dist(nn, newnode)
    newnode.parent = nn
    print ("parent", nn.x, nn.y)
    print ("newnode", newnode.x, newnode.y)
    return newnode, nn

def reWire(nodes,newnode):
    # Rewire the tree    
    
    for p in nodes:
        #if p!=newnode.parent and dist(p, newnode) < RADIUS and (newnode.cost + dist(p , newnode)) < p.cost:
        if p!=newnode.parent and p!= newnode and (newnode.cost + dist(p , newnode)) < p.cost: #The full rewiring thingy must be expensive computationally
            if not checkIntersect(newnode, p):
                p.parent = newnode
                p.cost = newnode.cost + dist(p, newnode)                       
    return nodes

def drawSolutionPath(start_pos, goal_pos, nn):    
    # To get the solution path out and show the solution in animation
    
    global goal
    global pose
    global boxId
    
    #Get the solution and show on tree
    solution = []
    solution.append(goal_pos)    
    while nn!=start_pos:  
        solution.insert(0, nn)      
        plt.plot(nn.x, nn.y, 'ro')
        plt.draw()
        nn = nn.parent
    solution.insert(0, start_pos)
    
    #Show solution
    #connect to GUI or DIRECT
    PBT.connect(PBT.GUI)
    #Set gravity
    PBT.setGravity(0,0,-10)
    #Import ground plane
    PBT.loadURDF("plane.urdf", [12, 12, 0]) 
    #Import obstacle boxes
    obsboxStartPos = [5, 2, 0]
    obsboxStartOrientation = PBT.getQuaternionFromEuler([0,0,0])
    PBT.loadURDF("box/urdf/box.urdf",obsboxStartPos, obsboxStartOrientation)
    obsboxStartPos = [5, 20, 0]
    obsboxStartOrientation = PBT.getQuaternionFromEuler([0,0,0])
    PBT.loadURDF("box/urdf/box.urdf",obsboxStartPos, obsboxStartOrientation)       
    #Import HMMR at desired pos
    cubeStartPos = [start_pos.x, start_pos.y, 0.14]
    cubeStartOrientation = PBT.getQuaternionFromEuler([0.0, 0.0, start_pos.theta])
    boxId = PBT.loadURDF("HMMR/urdf/HMMR.urdf",cubeStartPos, cubeStartOrientation)
    pose = [start_pos.x, start_pos.y, start_pos.theta, start_pos.theta1, start_pos.theta2]                
    
    input("Set zoom press enter")
    print ("Solution: ")
    for i in range(len(solution)):
        print (solution[i].x, solution[i].y, solution[i].theta, solution[i].theta1, solution[i].theta2)
        goal = [solution[i].x, solution[i].y, solution[i].theta, solution[i].theta1, solution[i].theta2]
        while  not at_goal():          
            gtg()          
            pose_update()  
            time.sleep(0.1)    
        pub_vel(0.0, 0.0)
    
    PBT.removeBody(boxId)        
    PBT.disconnect()
    
        
class Node:
    # The class for cost
    
    x = 0.0
    y = 0.0
    theta = 0.0
    theta1 = 0.0
    theta2 = 0.0
    cost = 0.0  
    parent = None
    def __init__(self, xcoord, ycoord, thetacrd, theta1crd, theta2crd):
        self.x = xcoord
        self.y = ycoord
        self.theta = thetacrd
        self.theta1 = theta1crd
        self.theta2 = theta2crd

    
def main():
    # The main function
    
    nodes = []# The tree of nodes
    
    nodes.append(Node(0.0, 0.0, 0.0, 0.0, 0.0)) # Start somewhere
    start_pos = nodes[0]
    start_pos.parent = start_pos #for plot purposes
    goal_pos = Node(10.0, 12.0, 0.0, 0.0, 0.0) # These needs to be better defined probably as functions
    
    plt.axis([0, 100, 0, 100])
    plt.plot(start_pos.x, start_pos.y, 'go', ms = 10.0)
    plt.plot(goal_pos.x, goal_pos.y, 'go', ms = 10.0)
    
    #Will have to some how adress the fact that the rotational degrees of freedom loop back
    #One solution is to allow for multiple complete revolutions within the provided window 
    #but still enable the goal check thing to identify that all the repeated goals locations are the same.
    
    flags = 0 #Meaning no solution found 
    while len(nodes) < NUMNODES:
        print (len(nodes))
        random.seed()
        rand_n = Node(random.random()*Window_size, random.random()*Window_size, 0.0, 0.0, 0.0) #rand_n is randomly sampled node        
        print ("rand", rand_n.x, rand_n.y, rand_n.theta)
        
        flagc = 1 #to check if there was collsion
        nn = nodes[0] #nn is nearest node to rand_n 
               
        for p in nodes:                                                
            if dist(p, rand_n) <= dist(nn, rand_n):
                if not checkIntersect(p, rand_n):
                    nn = p
                    flagc = 0 #Meaning no collission happened
                
        if flagc == 1:
            nn = nodes[0]
            for p in nodes:
                print ("second", p.x, p.y)
                if dist(p, rand_n) <= dist(nn, rand_n):
                    nn = p
            rand_n =  step_from_to(nn,rand_n)
            print ("step", p.x, p.y)
            print ("rand_n", rand_n.x, rand_n.y)
            if rand_n == nn: #Meaninng that node was of no use                
                continue
                                                    
        #Choose parent and rewire
        [newnode, nn] = chooseParent(nn, rand_n, nodes)
        nodes.append(newnode)
        print ("newnode", newnode.x, newnode.y, newnode.theta) 
        plt.plot(newnode.x, newnode.y, 'bo', ms = 4.0)                
        nodes = reWire(nodes, newnode) 
        
        if len(nodes)%(NUMNODES/10) == 0:
            nn = nodes[0]
            for p in nodes:
                if dist(p, goal_pos) < dist(nn, goal_pos):
                    nn = p       
            if not checkIntersect(nn, goal_pos):                 
                flags = 1
                break
            
    if flags == 1:
        PBT.disconnect()
        drawSolutionPath(start_pos, goal_pos, nn)
    else:
        print("Sorry, no solution")
    
    #Show the tree
    for p in nodes:
        plt.plot([p.x, p.parent.x], [p.y, p.parent.y], 'b--')          
    plt.show()
                                
    
    
# Finally we should run
if __name__ == '__main__':
    main()

#Guess there is no easy way out 
#The one option we have is to store up all the already done physics checks and run it out based on whenever you need_symbol
#If new only then go to do the check