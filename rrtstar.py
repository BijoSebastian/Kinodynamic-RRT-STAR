#!/usr/bin/env python

# rrtstar.py
# This program generates a 
# asymptotically optimal rapidly exploring random tree (RRT* proposed by Sertac Keraman, MIT) in a rectangular region.
#
# Originally written by Steve LaValle, UIUC for simple RRT in
# May 2011
# Modified by Md Mahbubur Rahman, FIU for RRT* in
# January 2016
# Modified by Bijo Sebastian, VT for Kinodynaic RRT* in
# August 2017

import random
from math import sqrt
import matplotlib.pyplot as mp

#parameters
Window_size = 100.0 #The operating window for each D.O.F
EPSILON = 7.0 #change
RADIUS = 15.0 #change
NUMNODES = 2000 #increase

def checkIntersect(p1, p2):
    #print (p1.x, p1.y, p1.z)
    #print (p2.x, p2.y, p2.z)
    
    # To check if intersections occur
    return False
    
def dist(p1,p2):
    # Measure euclidean distance between nodes
    return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z))

def step_from_to(p1,p2):
    # Grow from current node to destination node as far as you can before collision. 
    # Get the obs info as well and find the collision point 
    return p2

def chooseParent(nn,newnode,nodes):
    # Choose the new parent among the neighbors within radius only of there is no collission provided distance from start is lesser
    for p in nodes:
        if not checkIntersect(p, newnode) and dist(p, newnode) <RADIUS and (p.cost+dist(p, newnode)) < (nn.cost+dist(nn, newnode)):
            nn = p
    newnode.cost = nn.cost + dist(nn, newnode)
    newnode.parent = nn
    return newnode, nn

def reWire(nodes,newnode):
    # Rewire the tree    
    for p in nodes:
        #if not checkIntersect(p, newnode) and p!=newnode.parent and dist(p, newnode) < RADIUS and (newnode.cost + dist(p , newnode)) < p.cost:
        if not checkIntersect(p, newnode) and p!=newnode.parent and (newnode.cost + dist(p , newnode)) < p.cost: #The full rewiring thingy must be expensive computationally
            p.parent = newnode
            p.cost = newnode.cost + dist(p, newnode)                       
    return nodes

def drawSolutionPath(start, goal, nodes):    
    # To get the solution path out
    nn = nodes[0]
    for p in nodes:
        mp.plot([p.x, p.parent.x], [p.y, p.parent.y], 'b--')
        if dist(p, goal) < dist(nn, goal):
            nn = p            
    while nn!=start:
        print (nn.x, nn.y, nn.z)
        mp.plot(nn.x, nn.y, 'ro')
        mp.draw()
        nn = nn.parent
    print (nn.x, nn.y, nn.z)    
    mp.show()
        
class Node:
    # The class for cost
    x = 0.0
    y = 0.0
    z = 0.0
    cost = 0.0  
    parent = None
    def __init__(self, xcoord, ycoord, zcoord):
        self.x = xcoord
        self.y = ycoord
        self.z = zcoord
    
def main():
    # The main_old function
    nodes = []# The tree of nodes
    
    nodes.append(Node(0.0, 0.0, 0.0)) # Start somewhere
    start = nodes[0]
    start.parent = start #for plot purposes
    goal = Node(0.0, 20.0, 0.0) # These needs to be better defined probably as functions
    
    mp.axis([0, 100, 0, 100])
    mp.plot(start.x, start.y, 'go', ms = 10.0)
    mp.plot(goal.x, goal.y, 'go', ms = 10.0)
    
    #Will have to some how adress the fact that the rotational degrees of freedom loop back
    #One solution is to allow for multiple complete revolutions within the provided window 
    #but still enable the goal check thing to identify that all the repeated goals locations are the same. 
    for i in range(NUMNODES):
        print (i)
        rand = Node(random.random()*Window_size, random.random()*Window_size, random.random()*Window_size) #rand is randomly sampled node
        
        flagc = 0
        nn = nodes[0] #nn is nearest node to rand        
        for p in nodes:
            if dist(p, rand) < dist(nn, rand) and not checkIntersect(p, rand):
                nn = p
                flagc = 1
        if flagc == 0:
            nn = nodes[0]
            for p in nodes:
                if dist(p, rand) < dist(nn, rand):
                    nn = p
            rand =  step_from_to(p,rand)
                                                    
        #Choose parent and rewire
        [newnode, nn] = chooseParent(nn, rand, nodes)
        nodes.append(newnode)   
        mp.plot(newnode.x, newnode.y, 'bo', ms = 0.1)                
        nodes = reWire(nodes, newnode)                    

    drawSolutionPath(start, goal, nodes)
    
    
# Finally we should run
if __name__ == '__main__':
    main()
  



