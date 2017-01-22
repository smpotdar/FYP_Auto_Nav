import cv2
import numpy as np
from math import *

# create video capture
cap = cv2.VideoCapture(1)

def get_angle(bot_tip,bot_centre):
	x_vec=bot_tip[0]-bot_centre[0]
	y_vec=bot_tip[1]-bot_centre[1]
	if(x_vec >= 0):
		#bot_orient=atan2(bot_tip[1]-bot_centre[1],bot_tip[0]-bot_centre[0])
		bot_orient=atan2(bot_tip[0]-bot_centre[0],bot_tip[1]-bot_centre[1])
	else:	
		#bot_orient=2*pi+atan2(bot_tip[1]-bot_centre[1],bot_tip[0]-bot_centre[0])
		bot_orient=2*pi+atan2(bot_tip[0]-bot_centre[0],bot_tip[1]-bot_centre[1])	
	#return degrees((bot_orient+(5*pi/2))%(2*pi))
	#return ((2*pi-bot_orient)%2*pi)+(pi/2)
	return bot_orient



while(1):

    # read the frames
    _,frame = cap.read()

    # smooth it
    frame = cv2.blur(frame,(3,3))

    # convert to hsv and find range of colors
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    #thresh = cv2.inRange(hsv,np.array((0, 80, 80)), np.array((20, 255, 255)))
    thresh = cv2.inRange(hsv,np.array((110, 50, 50)), np.array((130, 255, 255)))
    thresh2 = thresh.copy()

    # find contours in the threshold image
    img,contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

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
    	cv2.circle(frame,(cx,cy),5,(0,0,255),-1)
    	font = cv2.FONT_HERSHEY_SIMPLEX
    	cv2.putText(frame,"Tip:"+str((cx,cy)),(cx,cy), font, 1,(255,0,0),2,cv2.LINE_AA)

    	S = cv2.moments(best_second_cnt)
    	dx,dy = int(S['m10']/S['m00']), int(S['m01']/S['m00'])
    	cv2.circle(frame,(dx,dy),5,(0,0,255),-1)
    	font = cv2.FONT_HERSHEY_SIMPLEX
    	cv2.putText(frame,"Centre:"+str((dx,dy)),(dx,dy), font, 1,(255,0,0),2,cv2.LINE_AA)

    	bot_orient=get_angle([cx,cy],[dx,dy])
    	cv2.putText(frame,"Bot_orient:"+str(round(bot_orient,2)),(300,30), font, 1,(0,255,0),2,cv2.LINE_AA)
	
    	cv2.line(frame, (dx,dy), (cx,cy), (0,255,0), 2)
    except:
	pass
	
    # Show it, if key pressed is 'Esc', exit the loop
    cv2.imshow('frame',frame)
    cv2.imshow('thresh',thresh2)
    if cv2.waitKey(33)== 27:
        break

# Clean up everything before leaving
cv2.destroyAllWindows()
cap.release()
