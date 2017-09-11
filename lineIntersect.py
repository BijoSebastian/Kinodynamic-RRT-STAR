def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD do not intersect??????????????????
#In our case this would be be a report from the bullet side that says if the position can actually be attained by the robot
#In whihc case returm true of position can be attianed and false otherwise
def checkIntersect(nodeA,nodeB,OBS):
    A=(nodeA.x,nodeA.y)
    B=(nodeB.x,nodeB.y)
    for o in OBS:
      obs=(o[0],o[1],o[0]+o[2],o[1]+o[3])
      C1=(obs[0],obs[1])
      D1=(obs[0],obs[3])
      C2=(obs[0],obs[1])
      D2=(obs[2],obs[1])
      C3=(obs[2],obs[3])
      D3=(obs[2],obs[1])
      C4=(obs[2],obs[3])
      D4=(obs[0],obs[3])
      inst1= ccw(A,C1,D1) != ccw(B,C1,D1) and ccw(A,B,C1) != ccw(A,B,D1) 
      inst2= ccw(A,C2,D2) != ccw(B,C2,D2) and ccw(A,B,C2) != ccw(A,B,D2)
      inst3= ccw(A,C3,D3) != ccw(B,C3,D3) and ccw(A,B,C3) != ccw(A,B,D3)
      inst4= ccw(A,C4,D4) != ccw(B,C4,D4) and ccw(A,B,C4) != ccw(A,B,D4)
      if inst1==False and inst2==False and inst3==False and inst4==False:
        #print(A,B)
        #input("Press Enter to continue...")
        continue      
      else:
         return False
    return True
def checkIntersectPoints(x,y,a,b,OBS):
    A=(x,y)
    B=(a,b)
    for o in OBS:
      obs=(o[0],o[1],o[0]+o[2],o[1]+o[3])
      C1=(obs[0],obs[1])
      D1=(obs[0],obs[3])
      C2=(obs[0],obs[1])
      D2=(obs[2],obs[1])
      C3=(obs[2],obs[3])
      D3=(obs[2],obs[1])
      C4=(obs[2],obs[3])
      D4=(obs[0],obs[3])
      inst1= ccw(A,C1,D1) != ccw(B,C1,D1) and ccw(A,B,C1) != ccw(A,B,D1) 
      inst2= ccw(A,C2,D2) != ccw(B,C2,D2) and ccw(A,B,C2) != ccw(A,B,D2)
      inst3= ccw(A,C3,D3) != ccw(B,C3,D3) and ccw(A,B,C3) != ccw(A,B,D3)
      inst4= ccw(A,C4,D4) != ccw(B,C4,D4) and ccw(A,B,C4) != ccw(A,B,D4)
      if inst1==False and inst2==False and inst3==False and inst4==False:
        #print(A,B)
        #input("Press Enter to continue...")
        continue      
      else:
         return False
    return True
