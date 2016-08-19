#!/usr/bin/env python
import cv2
import numpy as np
from zeabus_vision_srv.srv import vision_srv
from zeabus_vision_srv.msg import vision_msg
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import rospy
from operator import itemgetter
from vision_lib import *

frame = None
rows = None
cols = None
mask = None
task = 'bouy'
req = 'yellow'
i,j = 0,0 
channel = 1
lx,ly = 0,0
width = 320
height = 256

def not_found(m):
	m.x = lx
	m.y = ly
	m.area = 0
	m.appear = False

def getMask():
	global frame,task
	mask = None	
	hsv = process_img_top(frame.copy())
	# hsv = equalization(img.copy())
	# bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
	# hsv =  clahe(bgr)
	if task == 'gate':
		lower,upper = getColor('orange')
		mask = cv2.inRange(hsv,lower,upper)
	elif task == 'navigate':
		#lower =
		#upper = 
		lower,upper = getColor('yellow')
		mask = cv2.inRange(hsv,lower,upper)
	else :
		print 'request is false'
	return mask

def cam_callback(msg):
	global frame,rows,cols,req,i,j

	arr = np.fromstring(msg.data,np.uint8)
	frame = cv2.resize(cv2.imdecode(arr,1),(width,height))
	rows,cols,_ = frame.shape
	
def mission_callback(msg):
	global task,req
	task = msg.task.data
	req = msg.req.data
	if task == 'gate':
		output = do_gate(msg)
	elif task == 'navigate':
		output = do_navigate(msg)

	return output

def do_gate(msg):
	global frame,rows,cols,mask,task,req,lx,ly
	
	task = msg.task.data
	req = msg.req.data
	m = vision_msg()

	while frame is None or rows != height:
		# print 'not frame',rows
		rospy.sleep(0.01)

	# cv2.namedWindow('result',flags=cv2.WINDOW_NORMAL)
	# cv2.resizeWindow('result',width,height)

	mask = getMask()
	h, w = mask.shape
	
	# mask = cut_frame_top(mask)
	dilate = cv2.dilate(mask.copy(),np.array([[1]*5]*5))
	_,contours,hierarchy = cv2.findContours(dilate.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

	result = dilate.copy()
	result = cv2.cvtColor(result,cv2.COLOR_GRAY2BGR)
	
	sq_found =[]
	for c in contours:
		M = cv2.moments(c)
		x,y,ww,hh = cv2.boundingRect(c)
		
		area = cv2.contourArea(c)
		if  area > 100 and (y > 3 and y+hh < h-3 and x > 3 and x+ww < w-3) and hh/ww > 2:
			cv2.rectangle(result,(x,y),(x+ww,y+hh),(255),2)
			sq_found.append([ww*hh,
		    	             (x+(ww/2),
		        	          y+(hh/2)),hh])
			
	sq_found = sorted(sq_found,key = itemgetter(2))

	if len(sq_found) == 0:
		not_found(m)

	elif len(sq_found) == 1:
		x1, y1 = sq_found[-1][1] 
		x = x1
		y = y1 

		cv2.circle(result, (int(x),int(y)), 2, (0,0,255), -1)
		
		m.x = ((height/2.0) - x) / (height/2.0)
		m.y = ((width/2.0) - y) / (width/2.0)	

		m.area = sq_found[-1][0] / ((w*h)*1.0)
		m.appear = True
		lx, ly = m.x, m.y
	else :
		x1, y1 = sq_found[-1][1] 
		x2, y2 = sq_found[len(sq_found)-2][1] 
		x = (x1+x2) / 2.0
		y = (y1+y2) / 2.0 

		cv2.circle(result, (int(x1),int(y1)), 2, (0,255,0), -1)
		cv2.circle(result, (int(x),int(y)), 2, (0,0,255), -1)
		cv2.circle(result, (int(x2),int(y2)), 2, (0,255,0), -1)
		
		m.x = ((height/2.0) - x) / (height/2.0)
		m.y = ((width/2.0) - y) / (width/2.0)	

		m.area = (sq_found[-1][0] + sq_found[len(sq_found)-2][0]) / ((w*h)*1.0)
		m.appear = True
		lx, ly = m.x, m.y
	
	#cv2.imshow('result',result)
	k = cv2.waitKey(1) & 0xff
	if k == ord('q'):
		rospy.signal_shutdown('')
	
	#print m
	return m

def do_navigate(msg):
	global frame,rows,cols,mask,task,req,lx,ly
	
	m = vision_msg()
	task = msg.task.data
	req = msg.req.data

	while frame == None or rows > height:
		rospy.sleep(0.01)

	#cv2.namedWindow('result',flags=cv2.WINDOW_NORMAL)
	#cv2.resizeWindow('result',width,height)

	mask = getMask()
	h, w = mask.shape

	# mask = cut_frame_top(mask)
	erode = cv2.erode(mask,np.array([[1]*3]*3))
	dilate = cv2.dilate(mask,np.array([[1]*5]*5))

	result = dilate.copy()
	result = cv2.cvtColor(result,cv2.COLOR_GRAY2BGR)
	
	_,contours,hierarchy = cv2.findContours(dilate.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
	sq_found =[]

	for c in contours:
		M = cv2.moments(c)
		x,y,ww,hh = cv2.boundingRect(c)

		real_area = cv2.contourArea(c)
		area = ww*hh
		ratio = real_area/area
		# print real_area
		# print real_area, ww
		ratio_scale = ww/hh
		# print ratio, ratio_scale
		if  cv2.contourArea(c) > 100 and (y > 1 and y+hh < h-1 and x > 1 and x+ww < w-1 or (ratio_scale <= 2.5 and ratio < 0.3) or (ratio_scale > 2.5 and ratio > 0.5 and hh < 30) ) and ww > 70 :
			# print '>>',ratio, ratio_scale, hh
			cv2.rectangle(result,(x,y),(x+ww,y+hh),(0,255,0),2)
			cv2.circle(result,(int(x),int(y)), 5, (0,255,255), -1)	
			if h < 30 :
				sq_found.append([ww*hh,(x+(ww/2),y+(hh/2)),ww])
			else :
				sq_found.append([cv2.contourArea(c),(x+(ww/2),y+hh-10),ww])
			
	sq_found = sorted(sq_found,key = itemgetter(2))
	
	
	
	if len(sq_found) == 0:
		not_found(m)
	else :		
		x,y = sq_found[-1][1]
	
		cv2.circle(result,(int(x),int(y)), 5, (0,0,255), -1)
		
		m.y = ((h/2.0) - y) / (h/2.0)
		m.x = ((w/2.0) - x) / (w/2.0)	
		m.area = sq_found[-1][0]*1.0 / (w*h)	
		m.appear = True
		lx, ly = m.x, m.y
	
	#cv2.imshow('result',result)
	rospy.sleep(0.1)
	k = cv2.waitKey(1) & 0xff
	if k == ord('q'):
		rospy.signal_shutdown('')
	
	#print m
	return m

if __name__ == "__main__":
	rospy.init_node('vision_top')
	# rospy.Subscriber('/rightcam_top/image_raw/compressed',CompressedImage,cam_callback)
	rospy.Subscriber('/leftcam_top/image_raw/compressed',CompressedImage,cam_callback)
	# task = String('gate')
	# req = String('orange')
	# task = String('navigate')
	# req = String('yellow')
	# msg = vision_srv()
	# msg.task = task
	# msg.req = req
 	# while not rospy.is_shutdown():
	# 	do_navigate(msg)

	rospy.Service('vision1',vision_srv, mission_callback)
	rospy.spin()
