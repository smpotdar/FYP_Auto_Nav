import cv2
import numpy as np
from math import *
import socket
import random
import simplejson
from copy import deepcopy
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse
import imutils,sys

# create video capture
#cap = cv2.VideoCapture(0)

def get_angle(bot_tip,bot_centre):
	x_vec=bot_tip[0]-bot_centre[0]
	y_vec=bot_tip[1]-bot_centre[1]
	if(x_vec >= 0):
		bot_orient=atan2(bot_tip[0]-bot_centre[0],bot_tip[1]-bot_centre[1])
	else:	
		bot_orient=2*pi+atan2(bot_tip[0]-bot_centre[0],bot_tip[1]-bot_centre[1])	
	return bot_orient


def print_matrix(matrix):
	for i in range(len(matrix)):
		print matrix[i]

def get_euclidean_distance(x1,y1,x2,y2):
    euc_distance=sqrt(pow((y2-y1),2)+pow((x2-x1),2))
    return euc_distance

def get_new_pos(x_init,y_init,steer_alpha,bot_theta,d,l):
	beta=(d/float(l))*tan(radians(steer_alpha))  #Turning angle
	if(abs(beta)>0.001):
		R=d/float(beta)    # Turning radius
		x_new=x_init-sin(bot_theta)*R+sin(bot_theta+beta)*R
		y_new=y_init+cos(bot_theta)*R-cos(bot_theta+beta)*R
		theta_new=(bot_theta+beta)%(2*pi)
	else:
		x_new=x_init+d*cos(bot_theta)
		y_new=y_init+d*sin(bot_theta)
		theta_new=(bot_theta+beta)%(2*pi)
	
	return x_new,y_new,theta_new
	


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

def get_delta(x2,y2,theta_2,x_grid,y_grid,expand_2,expand_3):
	delta_in=0	
	for i in range(len(expand_2[x_grid][y_grid])):
		diff_x=expand_3[x_grid][y_grid][i][0]-x2
		diff_y=expand_3[x_grid][y_grid][i][1]-y2
		diff_theta=expand_3[x_grid][y_grid][i][2]-theta_2
		if(abs(diff_x)<0.001 and abs(diff_y<0.001) and abs(diff_theta)<0.001):
			delta_in=expand_2[x_grid][y_grid][i][0]

	return delta_in
				
				



def astar_search(grid,goal,init_cor,phi,cost,heuristic):
    # ----------------------------------------
    # modify the code below
    # ----------------------------------------
    #divisions=40
    divisions=20
    x = init_cor[0]
    y = init_cor[1]
    g = 0
    f=0
    init=[int(x/divisions),int(y/divisions)]
    delta = [[-35,1,1],[0,1,1],[35,1,1],[-35,-1,5],[0,-1,5],[35,-1,5]] # go right   
    #d=45  #moving distance
    #l=50  #Length of the car.
    d=23  #moving distance
    l=25  #Length of the car.
    theta=phi  #Orientation of the car
    alpha=0  #Steering Angle
    error=0
    error_2=0
    error_counter=0
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))] #Grid of closed cells
    counter_closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1
    dis=100

    found_path=[]
    found_path_2=[]	
    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]   #Stores expansions needed to reach at that particular cell
    expand_2=[[[] for row in range(len(grid[0]))] for col in range(len(grid))]   #Stores the delta move needed to reach at that particular cell
    expand_3=[[[] for row in range(len(grid[0]))] for col in range(len(grid))] #Stores the expanded cell coordinates and angles.
    
    
    open = [[f,g, x, y,theta]]
    
    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    computations=0
    closed_cells=[]
    while not found and not resign:
	#print "Main while entered"
        if len(open) == 0:
            resign = True
            #print "Motion failed"
            error=1
	    break
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[2]
            y = next[3]
	    theta=next[4]
            g = next[1]            
            x_grid=int(x/divisions)
	    y_grid=int(y/divisions)
	    expand[x_grid][y_grid] = count
            count += 1
	    if x_grid == goal[0] and y_grid == goal[1]:
                found = True
            else:
		#print "Expansion Started"
                for i in range(len(delta)):		                
		    x2,y2,theta_2=get_new_pos(x,y,delta[i][0],theta,delta[i][1]*d,l)
		    if(x2>=0 and y2>=0):
		    	x_grid=int(x2/divisions)
		    	y_grid=int(y2/divisions)
                    	if x_grid >= 0 and x_grid < len(grid) and y_grid >=0 and y_grid < len(grid[0]):
                        	if closed[x_grid][y_grid] == 0 and grid[x_grid][y_grid] == 0:
                        	    g2 = g + delta[i][2]+cost[x_grid][y_grid]
                        	    f2= g2+heuristic[x_grid][y_grid]				    
                        	    expand_2[x_grid][y_grid].append([i,counter_closed[x_grid][y_grid]])
				    expand_3[x_grid][y_grid].append([x2,y2,theta_2])
				    counter_closed[x_grid][y_grid]+=1
				    computations=computations+1
                        	    open.append([f2,g2, x2, y2,theta_2])
				    if x_grid == goal[0] and y_grid == goal[1]:
                			found = True
		
		for q in range(len(counter_closed)):
			for r in range(len(counter_closed[0])):
				if(counter_closed[q][r]>0):		                  
					closed[q][r] = 1
					
     
    #print_matrix(expand)
    #print_matrix(expand_2)		
    if(error==0):
	[x,y,theta]=expand_3[goal[0]][goal[1]][0]
    	x_grid=int(x/divisions)
    	y_grid=int(y/divisions)
    	[delta_in,in_index]=expand_2[x_grid][y_grid][0] #will give you the final delta action and the index of chosen delta action
    	found_path.append([x,y,delta_in])
	found_path_2.append([x,y])
    	while(x_grid!=init[0] or y_grid!=init[1]):		
		x2,y2,theta_2=get_new_pos(x,y,delta[delta_in][0],theta,(-1)*delta[delta_in][1]*d,l)    	
		x_grid=int(x2/divisions)
   	        y_grid=int(y2/divisions)
		error_counter+=1
		#if(error_counter>30):
		if(error_counter>50):
			error_2=1
			#print_matrix(found_path_2)
			break
		delta_in=get_delta(x2,y2,theta_2,x_grid,y_grid,expand_2,expand_3)
		found_path.append([x2,y2,delta_in])
		found_path_2.append([x2,y2])
		dis=get_euclidean_distance(init_cor[0],init_cor[1],x2,y2)
		#if(dis<5):
		if(dis<10):
			#print "reached"
			break		
		#print delta_in
        	x=x2
        	y=y2
		theta=theta_2
		

    print "Computations:",computations
    return list(reversed(found_path_2)),list(reversed(found_path)),error,error_2



lid_data=[]


#TCP_IP = '127.0.0.1'
TCP_IP = str(sys.argv[1])
TCP_PORT = 5005
BUFFER_SIZE = 16384
MESSAGE = "Send Data"
END_MSG="band kar"
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
vs = WebcamVideoStream(src=1).start()
frame = vs.read()
height,width=frame.shape[:2]
#divisions=40
divisions=20
h=height/divisions
w=width/divisions
map_matrix=[[0 for rows in range(w)] for cols in range(h)]
cost_map=[[ 1 for rows in range(w)] for cols in range(h)]
cost_move=[[-1, 0], # go up
           [ 0,-1], # go left
       	   [ 1, 0], # go down
           [ 0, 1],
	   [ 1, 1],
	   [-1, 1],
	   [-1,-1],
	   [ 1, -1]] # go right
goal = [len(map_matrix)-1, len(map_matrix[0])-1]
path_2=[]
path_3=[]	
phi=pi
#l=50
l=25
cost=1
a=0
counter=0
while(1):
    	counter+=1
	if(counter>50):	
    		s.send(MESSAGE)
    		data = s.recv(BUFFER_SIZE)
    		data_2=simplejson.loads(data)
    		lid_data=deepcopy(data_2)
    	#for i in range(360):    #p[i]=[random.randrange(80,100,1)]
    	#	lid_data.append(random.randrange(1750,2000,1))
    	
	map_matrix=[[0 for rows in range(w)] for cols in range(h)]
    	cost_map=[[1 for rows in range(w)] for cols in range(h)]

    	# read the frames
   	# _,frame = cap.read()
    	frame = vs.read()
    	img=frame.copy()
    	img = cv2.blur(img,(3,3))
    	hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    	thresh = cv2.inRange(hsv,np.array((110, 50, 50)), np.array((130, 255, 255)))
    	thresh2 = thresh.copy()
    	img2,contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    # finding contour with maximum area and store it as best_cnt
    #best_cnt=0
	try:
    		max_area = 0
    		for cnt in contours:
        		area = cv2.contourArea(cnt)
        		if area > max_area:
        		    max_area = area
        		    best_cnt = cnt                       #Largest Contour

    		max_second_area=0
    		for cnt in contours:
        		area = cv2.contourArea(cnt)
        		if area > max_second_area and area != max_area:
        		    max_second_area = area
        		    best_second_cnt = cnt                #Second Largest Contour


    	# finding centroids of best_cnt and draw a circle there
    		M = cv2.moments(best_cnt)
    		cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
    		cv2.circle(img,(cx,cy),5,(0,0,255),-1)
    		font = cv2.FONT_HERSHEY_SIMPLEX
    		cv2.putText(img,"Tip:"+str((cx,cy)),(cx,cy), font, 1,(255,0,0),2,cv2.LINE_AA)

    		S = cv2.moments(best_second_cnt)
    		dx,dy = int(S['m10']/S['m00']), int(S['m01']/S['m00'])
    		cv2.circle(img,(dx,dy),5,(0,0,255),-1)
    		font = cv2.FONT_HERSHEY_SIMPLEX
    		cv2.putText(img,"Centre:"+str((dx,dy)),(dx,dy), font, 1,(255,0,0),2,cv2.LINE_AA)

    		bot_orient=get_angle([cx,cy],[dx,dy])
    		cv2.putText(img,"Bot_orient:"+str(round(bot_orient,2)),(300,30), font, 1,(0,255,0),2,cv2.LINE_AA)

    		cv2.line(img, (dx,dy), (cx,cy), (0,255,0), 2)

		init_cor=[cy,cx]
    	#x_mid=init_cor[0]+l*cos(phi)
    	#y_mid=init_cor[1]+l*sin(phi)
    	#x_mid=init_cor[0]+l*cos(bot_orient)
    	#y_mid=init_cor[1]+l*sin(bot_orient)	

    		for i in range(w+1):
			for j in range(h+1):
				cv2.circle(img,(i*divisions,j*divisions),2,(0,0,255),-1)
	
    	#Painting Laser Data
    		if(counter>100):
    			for i in range(len(lid_data)):
				if(lid_data[i][0]>=200 and lid_data[i][0]<=1250):
					data_theta=(bot_orient+radians(i))%(2*pi)
					y_lid_data=int(cy+(lid_data[i][0]/3.0)*cos(data_theta)/1.0)
					x_lid_data=int(cx+(lid_data[i][0]/3.0)*sin(data_theta)/1.0)
					x_matrix=x_lid_data/divisions
					y_matrix=y_lid_data/divisions
					if(x_matrix>=0 and y_matrix>=0 and x_matrix<len(map_matrix[0]) and y_matrix<len(map_matrix)):
						map_matrix[y_matrix][x_matrix]=1
						x1=x_matrix*divisions
						y1=y_matrix*divisions
						x2=x1+divisions
						y2=y1+divisions
						cv2.rectangle(img, (x1, y1), (x2, y2), (255,0,0), -1)
	
    		#if(counter>100):
    		#	for i in range(90,180):
		#		data_theta=(bot_orient+radians(i))%(2*pi)
		#		y_lid_data=int(cy+(lid_data[i]/10.0)*cos(data_theta)/1.0)
		#		x_lid_data=int(cx+(lid_data[i]/10.0)*sin(data_theta)/1.0)
		#		x_matrix=x_lid_data/divisions
		#		y_matrix=y_lid_data/divisions
		#		if(x_matrix>=0 and y_matrix>=0 and x_matrix<len(map_matrix[0]) and y_matrix<len(map_matrix)):
		#			map_matrix[y_matrix][x_matrix]=1
		#			x1=x_matrix*divisions
		#			y1=y_matrix*divisions
		#			x2=x1+divisions
		#			y2=y1+divisions
		#			cv2.rectangle(img, (x1, y1), (x2, y2), (255,0,0), -1)
    				#cv2.circle(img,(x_lid_data,y_lid_data),5,(255,255,255),-1)
			
			
		#	for i in range(181,270):
		#		data_theta=(bot_orient+radians(i))%(2*pi)
		#		y_lid_data=int(cy+(lid_data[i]/10.0)*cos(data_theta)/1.0)
		#		x_lid_data=int(cx+(lid_data[i]/10.0)*sin(data_theta)/1.0)
		#		x_matrix=x_lid_data/divisions
		#		y_matrix=y_lid_data/divisions
		#		if(x_matrix>=0 and y_matrix>=0 and x_matrix<len(map_matrix[0]) and y_matrix<len(map_matrix)):
		#			map_matrix[y_matrix][x_matrix]=1
		#			x1=x_matrix*divisions
		#			y1=y_matrix*divisions
		#			x2=x1+divisions
		#			y2=y1+divisions
		#			cv2.rectangle(img, (x1, y1), (x2, y2), (255,0,0), -1)
	

		for i in range(len(map_matrix)):
			for j in range(len(map_matrix[0])):
				if(map_matrix[i][j]==1):
					cost_map[i][j]=99
					for q in range(len(cost_move)):
        	        	    		q2 = i + cost_move[q][0]
        	        	    		r2 = j + cost_move[q][1]
        	        	    		if q2 >= 0 and q2 < len(cost_map) and r2 >=0 and r2 < len(cost_map[0]):
							cord=[q2,r2]
							if map_matrix[q2][r2]==0 and goal!=cord:
        	        	       				cost_map[q2][r2]=cost_map[q2][r2]+10	
    		for i in range(len(cost_map)):
			for j in range(len(cost_map[0])):
				if(cost_map[i][j]!=1 and cost_map[i][j]!=99):
					pix=int(cost_map[i][j]*255/float(90))
					x1=j*divisions
					y1=i*divisions
					x2=x1+divisions
					y2=y1+divisions
					cv2.rectangle(img, (x1, y1), (x2, y2), (pix,pix,pix), -1)
	
		print "A-Start Search:"
		#print "Map:"
		#print_matrix(map_matrix)
    		heuristic=get_heuristic(map_matrix,goal)
		#path_main,path,error,error_2=astar_search(map_matrix,goal,init_cor,phi,cost_map,heuristic)
		path_main,path,error,error_2=astar_search(map_matrix,goal,init_cor,bot_orient,cost_map,heuristic)
    		if(error==0):		
			path_3=deepcopy(path)
			newpath=smooth(path_main, weight_data = 0.5, weight_smooth = 0.5, tolerance = 0.000001)
			path_2=deepcopy(newpath)
			error=0		
    		else:
			print "Search failed"	

		if(path_2):
			for i in range(len(path_2)-1):
				x1=int(path_2[i][0])
				y1=int(path_2[i][1])
				x2=int(path_2[i+1][0])
				y2=int(path_2[i+1][1])		
				cv2.circle(img,(y1,x1),2,(255,255,255),-1)
				cv2.line(img, (y1,x1),(y2,x2), (255,255,255), 2)
				#cv2.circle(img,(x1,y1),2,(255,255,255),-1)
				#cv2.line(img, (x1,y1),(x2,y2), (255,255,255), 2)			
	
    		if(path_3):
			for i in range(len(path_3)-1):	
				x1=int(path_3[i][0])
				y1=int(path_3[i][1])
				x2=int(path_3[i+1][0])
				y2=int(path_3[i+1][1])
				if(path_3[i+1][2]>2):		
					cv2.circle(img,(y1,x1),2,(0,255,0),-1)
					cv2.line(img, (y1,x1),(y2,x2), (0,255,0), 2)
				else:
					cv2.circle(img,(y1,x1),2,(0,0,255),-1)
					cv2.line(img, (y1,x1),(y2,x2), (0,0,255), 2)

    		lid_data[:]=[]
    		# Show it, if key pressed is 'Esc', exit the loop
	except:
		pass    	
	cv2.imshow('frame',img)
    	#cv2.imshow('thresh',thresh2)
    	if cv2.waitKey(5)== 27:
    	    break

# Clean up everything before leaving
#s.send(END_MSG)
#s.close()
cv2.destroyAllWindows()
#cap.release()
