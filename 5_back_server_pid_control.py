import cv2
import numpy as np
from math import *
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse
import imutils
import socket,random,simplejson
import sys
# create video capture


def get_angle(bot_tip,bot_centre):
	x_vec=bot_tip[0]-bot_centre[0]
	y_vec=bot_tip[1]-bot_centre[1]
	if(x_vec >= 0):
		bot_orient=atan2(bot_tip[0]-bot_centre[0],bot_tip[1]-bot_centre[1])
	else:	
		bot_orient=2*pi+atan2(bot_tip[0]-bot_centre[0],bot_tip[1]-bot_centre[1])	
	return bot_orient

def get_cte_angle(bot_pos,target,phi_bot):
	phi_ref=get_angle(target,bot_pos)	
	diff=phi_bot-phi_ref
	if(diff<=radians(180.0)):
		return diff
	else:
		return diff-2*radians(180.0)

def get_per_distance(x_bot,y_bot,x1,y1,x2,y2):
    a=sqrt(pow((y2-y1),2)+pow((x2-x1),2))
    b=(y2-y1)*x_bot-(x2-x1)*y_bot+x2*y1-y2*x1
    distance=-b/a

    return distance

def get_euclidean_distance(x1,y1,x2,y2):
    euc_distance=sqrt(pow((y2-y1),2)+pow((x2-x1),2))
    return euc_distance

def get_cte(x_bot,y_bot,x1,y1,x2,y2):
    x_vec=x2-x1
    y_vec=y2-y1

    cte=0

    if(x_vec!=0 and y_vec!=0):
        distance=get_per_distance(x_bot,y_bot,x1,y1,x2,y2)
        cte=distance
    elif(x_vec > 0 and y_vec == 0):
        #print "Right Axis"
        cte=y_bot-y1
    elif(x_vec < 0 and y_vec == 0):
        #print "Left Axis"
        cte=-(y_bot-y1)
    elif(x_vec == 0 and y_vec < 0):
        #print "South Axis"
        cte=x_bot-x1
    else:
        #print "North Axis"
        cte=-(x_bot-x1)

    return cte

def get_cte_circle(x_bot,y_bot,centre,radius):    
    cte=0
    cte=get_euclidean_distance(x_bot,y_bot,centre[0],centre[1])-radius    
    return cte

def get_servo_ds(b):
        if(b>45.0):
                b=45.0
        if(b<-45.0):
                b=-45.0
        servo_dc=7.5-((1.3*b)/45.0)
        return round(servo_dc,1)

TCP_IP = str(sys.argv[1])
TCP_PORT = 5005
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
lid_data=[]
conn, addr = s.accept()
conn.settimeout(0.01)
counter=0
print 'Connection address:', addr
vs = WebcamVideoStream(src=1).start()
frame = vs.read()
drive_flag=1
path_count=0#path=[[5,250,1],[250,250,1],[450,450,1],[450,100,1],[200,200,1]]
path=[[600,450,3],[150,150,1],[600,450,3]]
sum_i=0
s=0
a=0
d=0
c=0
cx=0
cy=0
thread_counter=0
latch_path=0
cte_old=0
cte_new=0
z=0
tau_p=0.2 #OG
#tau_p=2.9
#tau_d=3.0 #OG
tau_d=6
#tau_i=0.49
data=0
while(1):
    thread_counter+=1
    # read the frames
    frame = vs.read()
    img=frame.copy()
    img = cv2.blur(img,(3,3))
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    #thresh = cv2.inRange(hsv,np.array((0, 80, 80)), np.array((20, 255, 255)))
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
    	bot_orient_opp=get_angle([dx,dy],[cx,cy])
    	cv2.putText(img,"Bot_orient:"+str(round(bot_orient,2)),(300,30), font, 1,(0,255,0),2,cv2.LINE_AA)
	
    	cv2.line(img, (dx,dy), (cx,cy), (0,255,0), 2)

    	for i in range(len(path)-1):	
		x1=int(path[i][0])
		y1=int(path[i][1])
		x2=int(path[i+1][0])
		y2=int(path[i+1][1])
		if(path[i+1][2]>2):		
			cv2.circle(img,(x1,y1),2,(0,255,0),-1)
			cv2.line(img, (x1,y1),(x2,y2), (0,255,0), 2)
		else:
			cv2.circle(img,(x1,y1),2,(0,0,255),-1)
			cv2.line(img, (x1,y1),(x2,y2), (0,0,255), 2)

    # Show it, if key pressed is 'Esc', exit the loop
    
    	path_length=len(path)
    	if(drive_flag==1 and thread_counter>5):	
	#print "Driving"	
		x1=path[path_count][0]
        	y1=path[path_count][1]
        	x2=path[path_count+1][0]
        	y2=path[path_count+1][1]
		gear=path[path_count+1][2]	
		print [x1,y1],"<--->",[x2,y2]
		cte_new=get_cte(cx,cy,x1,y1,x2,y2)        	
		diff_cte=cte_new-cte_old
		sum_i=sum_i+cte_new
       		#cte_old=cte_new
        	steering = -tau_p * cte_new - tau_d * diff_cte# - tau_i*sum_i
		if(gear<=2):
			euc_dist=get_euclidean_distance(cx,cy,x2,y2)
		        z=get_servo_ds(-1*steering)
			data_pwm=[z,30,1]
		if(gear>2):
			euc_dist=get_euclidean_distance(dx,dy,x2,y2)
		        z=get_servo_ds(steering)
			data_pwm=[z,30,2]	
		print z
		if(thread_counter>10):
			try:
				data = conn.recv(BUFFER_SIZE)
    				if data=="send":
					conn.send(simplejson.dumps(data_pwm))
					print "sent"
					data=0
				#break
			except:
				pass 
		#print "Data Sent."
		#cte_new=get_cte(cx,cy,x1,y1,x2,y2)
		cte_old=cte_new
		#thread_counter=0        
        	if(euc_dist<60):
        	        path_count=path_count+1
			sum_i=0
		if(path_count>=path_length):
			#drive_flag=0
			path_count=0
			break
    except:
	pass

    cv2.imshow('frame',img)
    cv2.imshow('thresh',thresh2)

    if cv2.waitKey(5)== 27:
        break

# Clean up everything before leaving
cv2.destroyAllWindows()
vs.stop()
