import cv2
import numpy as np
from copy import deepcopy
import math

def draw_circle(event,x,y,flags,param):
    #global ix,iy    
    if event == cv2.EVENT_LBUTTONDOWN:
	x_dash=x/divisions
	y_dash=y/divisions
	if(map_matrix[y_dash][x_dash]==0):
		map_matrix[y_dash][x_dash]=1		
		
	else:
		map_matrix[y_dash][x_dash]=0	
		

def print_matrix(matrix):
	for i in range(len(matrix)):
		print matrix[i]

def smooth(path, weight_data = 0.5, weight_smooth = 0.1, tolerance = 0.000001):

    # Make a deep copy of path into newpath
    newpath = deepcopy(path)
    change=0
    change_1=0
    change_2=0
  
    while True:    
        for i in range(1,len(path)-1):
            
            x1=newpath[i][0]
            y1=newpath[i][1]
            newpath[i][0]=newpath[i][0]+weight_data*(path[i][0]-newpath[i][0])+weight_smooth*(newpath[i+1][0]+newpath[i-1][0]-2*newpath[i][0])
            newpath[i][1]=newpath[i][1]+weight_data*(path[i][1]-newpath[i][1])+weight_smooth*(newpath[i+1][1]+newpath[i-1][1]-2*newpath[i][1])
            x2=newpath[i][0]
            y2=newpath[i][1]
            change=change+abs(x2-x1)+abs(y2-y1)            
        
        change_1=change
        delta=abs(change_2-change_1)
        change_2=change_1
        
        if(delta<tolerance):
            break
         
        if(delta>=tolerance):
            #print change
            change=0
            
    return newpath # Leave this line for the grader!

def get_heuristic(grid,goal):
    heuristic=[[0 for cols in range(len(grid[0]))] for rows in range(len(grid))]
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    delta = [[-1, 0 ], # go up
    	     [ 0, -1], # go left
    	     [ 1, 0 ], # go down
    	     [ 0, 1 ]] # go right
    closed[goal[0]][goal[1]]=1
    cost=1
    x=goal[0]
    y=goal[1]
    open=[]
    g=0
    heuristic[x][y]=g
    error=0
#    while(x!=0 or y!=0):
    while(True):
        for i in range(len(delta)):
            x2=x-delta[i][0]
            y2=y-delta[i][1]        
            if( x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0])):
		if closed[x2][y2] == 0:               
                    g=g+cost
                    open.append([g,x2,y2])
		    closed[x2][y2]=1
                    heuristic[x2][y2]=g
                    g=g-cost
        if(open):
	    #print open            
            next = open[0]
            open.remove(next)
            x=next[1]
            y=next[2]
            g=next[0]
        else:
            error=1
        
        if(error==1):
            break
        
    return heuristic


def astar_search(grid,init,goal,cost,heuristic):
    # ----------------------------------------
    # modify the code below
    # ----------------------------------------
    delta = [[-1, 0 ], # go up
             [ 0, -1], # go left
             [ 1, 0 ], # go down
             [ 0, 1 ]] # go right

    delta_name = ['^', '<', 'v', '>']

    error=0
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    found_path=[]
    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    expand_2=[[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    expand_3=[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    f=0
    open = [[f,g, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    computations=0
    while not found and not resign:
        if len(open) == 0:
            resign = True
            print "Motion failed"
            error=1
	    break
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[2]
            y = next[3]
            g = next[1]
            #f=  g+heuristic[x][y]
            #f=  next[0]
            expand[x][y] = count
            count += 1
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            f2= g2+heuristic[x2][y2]
                            expand_2[x2][y2]=i
			    computations=computations+1
                            open.append([f2,g2, x2, y2])
                            closed[x2][y2] = 1

    x=goal[0]
    y=goal[1]
    found_path.append([x,y])
    expand_3[x][y]='*'
    if(error==0):
    	while(x!=init[0] or y!=init[1]):
        	x2=x-delta[expand_2[x][y]][0]
        	y2=y-delta[expand_2[x][y]][1]
        	expand_3[x2][y2]=delta_name[expand_2[x][y]]
		found_path.append([x2,y2])
        	x=x2
        	y=y2

    print "Computations:",computations
    return expand_3,list(reversed(found_path)),error



def depth_search(grid,goal,init):
		
	cost = 1
	found_path=[]
	delta = [[-1, 0 ], # go up
        	 [ 0, -1], # go left
        	 [ 1, 0 ], # go down
        	 [ 0, 1 ]] # go right

	delta_name = ['^', '<', 'v', '>']
	
	closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    	closed[init[0]][init[1]] = 1
	computations=0
    	error=0
    	x = init[0]
    	y = init[1]
    	g = 0
    	expand_2=[[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    	expand_3=[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    	open = [[g, x, y]]

    	found = False  # flag that is set when search is complete
    	resign = False # flag set if we can't find expand
	while not found and not resign:
        	if len(open) == 0:
            		resign = True
            		#return 'fail' #Original
	    		print 'fail'
            		error=1
	    		break

        	else:
            		open.sort()
            		open.reverse()
            		next = open.pop()
            		x = next[1]
        	        y = next[2]
        	        g = next[0]
            
            		if x == goal[0] and y == goal[1]:
                		found = True
            		else:
                		for i in range(len(delta)):
                    			x2 = x + delta[i][0]
                    			y2 = y + delta[i][1]
                    			if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        			if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            				g2 = g + cost
                            				open.append([g2, x2, y2])
                            				closed[x2][y2] = 1
                            				expand_2[x2][y2]=i
							computations=computations+1
    		
	x=goal[0]
	y=goal[1]
    	found_path.append([x,y])
   	expand_3[x][y]='*'
    	if(error==0):
    		while(x!=init[0] or y!=init[1]):
        		x2=x-delta[expand_2[x][y]][0]
        		y2=y-delta[expand_2[x][y]][1]
        		expand_3[x2][y2]=delta_name[expand_2[x][y]]
			found_path.append([x2,y2])
        		x=x2
        		y=y2
        
    #return expand_3 #Original
	print "Computations:",computations
    	return expand_3,list(reversed(found_path)),error


def dynamic_program_search(grid,goal,cost):
    # ----------------------------------------
    # modify code below
    # ----------------------------------------

    value = [[99 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[goal[0]][goal[1]]=1
    delta = [[-1, 0 ], # go up
             [ 0, -1], # go left
             [ 1, 0 ], # go down
             [ 0, 1 ]] # go right

    delta_name = ['^', '<', 'v', '>']
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if(grid[i][j]==1):
                value[i][j]=99
    x=goal[0]
    y=goal[1]
    open=[]
    g=0
    value[x][y]=g
    error=0
#    while(x!=0 or y!=0):
    while(True):
        for i in range(len(delta)):
            x2=x-delta[i][0]
            y2=y-delta[i][1]        
            if( x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0])):
                if closed[x2][y2] == 0 and grid[x2][y2] == 0:                
                    g=g+cost
                    action[x2][y2]=i
                    closed[x2][y2]=1
                    open.append([g,x2,y2])
                    #print open
                    value[x2][y2]=g
                    g=g-cost
        if(open):            
            next = open[0]
            open.remove(next)
            x=next[1]
            y=next[2]
            g=next[0]
        else:
            error=1
        
        if(error==1):
            break
    
    policy[goal[0]][goal[1]]='*'
    for i in range(len(action)):
        for j in range(len(action[0])):
            if(action[i][j]>-1):
                policy[i][j]=delta_name[action[i][j]]

    return policy


cap = cv2.VideoCapture(1)	
ret, frame = cap.read()
height,width=frame.shape[:2]
cv2.namedWindow('Sankalp frame')
cv2.setMouseCallback('Sankalp frame',draw_circle)
divisions=40
h=height/divisions
w=width/divisions
map_matrix=[[0 for rows in range(w)] for cols in range(h)]
goal = [len(map_matrix)-1, len(map_matrix[0])-1]
path_2=[]
path_3=[]	
init = [0, 0]
cost=1
s=0
a=0
d=0
while(1):
    ret, frame = cap.read()
    for i in range(w+1):
	for j in range(h+1):
		cv2.circle(frame,(i*divisions,j*divisions),2,(0,0,255),-1)

    for i in range(len(map_matrix)):
	for j in range(len(map_matrix[0])):
		if(map_matrix[i][j]==1):
			x1=j*divisions
			y1=i*divisions
			x2=x1+divisions
			y2=y1+divisions
			cv2.rectangle(frame, (x1, y1), (x2, y2), (255,0,0), -1)
	
    if(path_2):
	for i in range(len(path_2)-1):
		x1=int(divisions*path_2[i][0]+(divisions/2))
		y1=int(divisions*path_2[i][1]+(divisions/2))
		x2=int(divisions*path_2[i+1][0]+(divisions/2))
		y2=int(divisions*path_2[i+1][1]+(divisions/2))		
		cv2.circle(frame,(y1,x1),2,(0,255,0),-1)
		cv2.line(frame, (y1,x1),(y2,x2), (0,255,0), 2)
	
    if(path_3):
	for i in range(len(path_3)-1):
		x1=int(divisions*path_3[i][0]+(divisions/2))
		y1=int(divisions*path_3[i][1]+(divisions/2))
		x2=int(divisions*path_3[i+1][0]+(divisions/2))
		y2=int(divisions*path_3[i+1][1]+(divisions/2))		
		cv2.circle(frame,(y1,x1),2,(0,0,255),-1)
		cv2.line(frame, (y1,x1),(y2,x2), (0,0,255), 2)		
		
							
				
    cv2.imshow('Sankalp frame',frame)	
    #cv2.imshow('image',img)	
    if cv2.waitKey(5) & 0xFF == ord('s'):
	s=1
    
    if(s==1):
	print "Depth Search:"
	print "Map:"
	#print_matrix(map_matrix)
	expand,path,error=depth_search(map_matrix,goal,init)
	if(error==0):
		print "Directions:"
		print_matrix(expand)		
		path_3=deepcopy(path)
		newpath=smooth(path, weight_data = 0.5, weight_smooth = 0.5, tolerance = 0.000001)
		path_2=deepcopy(newpath)		
		#print newpath
		error=0		
	else:
		print "Search failed"	
	
	s=0

    if cv2.waitKey(5) & 0xFF == ord('a'):
	a=1
    
    if(a==1):
	print "A-Start Search:"
	print "Map:"
	#print_matrix(map_matrix)
	heuristic=get_heuristic(map_matrix,goal)
	expand,path,error=astar_search(map_matrix,init,goal,cost,heuristic)
	if(error==0):		
		
		print "Directions:"
		print_matrix(expand)
		path_3=deepcopy(path)
		newpath=smooth(path, weight_data = 0.5, weight_smooth = 0.5, tolerance = 0.000001)
		path_2=deepcopy(newpath)
		error=0		
	else:
		print "Search failed"	
	
	a=0	

    if cv2.waitKey(5) & 0xFF == ord('d'):
	d=1
    
    if(d==1):
	print "Dynamic Programming Policy:"
	print "Map:"
	#print_matrix(map_matrix)
	expand=dynamic_program_search(map_matrix,goal,cost)
	error=0
	if(error==0):		
		
		print "Directions:"
		print_matrix(expand)
		path_3=deepcopy(path)
		newpath=smooth(path, weight_data = 0.5, weight_smooth = 0.5, tolerance = 0.000001)
		path_2=deepcopy(newpath)
		error=0		
	else:
		print "Search failed"	
	
	d=0	
    
    if cv2.waitKey(5) & 0xFF == 27:
        break

cv2.destroyAllWindows()
