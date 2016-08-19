#!/usr/bin/env python
import cv2
import numpy as np
import math
import rospy

def getColor(color):
	# Navigate
	if color == "yellow":
		lower = np.array(rospy.get_param('/color_range/color_top/lower_yellow'),np.uint8)
		upper = np.array(rospy.get_param('/color_range/color_top/upper_yellow'),np.uint8)
	# gate
	elif color == 'orange':
		lower = np.array(rospy.get_param('/color_range/color_top/lower_orange'),np.uint8)
 		upper = np.array(rospy.get_param('/color_range/color_top/upper_orange'),np.uint8)
	# path
	elif color == 'orange1':
		lower = np.array(rospy.get_param('/color_range/color_down/lower_orange1'),np.uint8)
		upper = np.array(rospy.get_param('/color_range/color_down/upper_orange1'),np.uint8)
	elif color == 'orange2':
		lower = np.array(rospy.get_param('/color_range/color_down/lower_orange2'),np.uint8)
		upper = np.array(rospy.get_param('/color_range/color_down/upper_orange2'),np.uint8)
	elif color == 'yellow1':
		lower = np.array(rospy.get_param('/color_range/color_down/lower_yellow'),np.uint8)
		upper = np.array(rospy.get_param('/color_range/color_down/upper_yellow'),np.uint8)
	# print lower,upper
	return lower,upper

def equalization(frame):
	img = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	h , s , v = cv2.split(img)

	equ_s = cv2.equalizeHist(s)
	equ_v = cv2.equalizeHist(v)

	equ = cv2.merge((h,equ_s,equ_v))
	
		# h, s, v = cv2.split(result_hsv)
	print h.max(),s.max(),v.max()
	
	return equ
def clahe(img):
	# ____________________________ #

	img = cv2.cvtColor(img,cv2.COLOR_BGR2Lab)
	L, a, b = cv2.split(img)
	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
 	L = clahe.apply(L)
 	
	result = cv2.merge((L,a,b))
	result_bgr = cv2.cvtColor(result,cv2.COLOR_Lab2BGR)
	result_hsv = cv2.cvtColor(result_bgr,cv2.COLOR_BGR2HSV)
	return result_hsv

def stretching(img):
	
	b , g , r = cv2.split(img)
	b -= b.min()
	b *= int(round(255.0 /( b.max() - b.min() )))
	g -= g.min()
	g *= int(round(255.0 /( g.max() - g.min() )))
	# r -= r.min()
	# r *= 255 /( r.max() - r.min())

	img = cv2.merge((b,g,r))
	img = cv2.cvtColor(img , cv2.COLOR_BGR2HSV)
	h , s , v = cv2.split(img)
	
	s -= s.min()
	s *= int(round(255.0 /( s.max() - s.min() )))
	
	v -= v.min()
	v *= int(round(255.0 /( v.max() - v.min() )))
	

	img = cv2.merge((h,s,v))
	img = cv2.cvtColor(img,cv2.COLOR_HSV2BGR)

	return img
def cut_frame_top(img):
	h, w = img.shape

	for i in range (0,h/4):
		for j in range (0,w):
			if (img[i,j] == 255):
				img[i,j] = 0
	return img

def cut_frame_down(img):
	h, w = img.shape

	for i in range (h*3/4,h):
		for j in range (0,w):
			if (img[i,j] == 255):
				img[i,j] = 0
	return img

def mask_longerline(img):
	h, w = img.shape
	d = dmax = row = 0

	for i in range(h/4,h):
		d = 0
		for j in range(0,w):
			if(img[i,j] == 255):
				d += 1
		if(d>dmax):
			row=i

	result = np.zeros((h,w),dtype=np.uint8)
	start = row-50
	last = row+50
	if last > h:
		last = h
	if start <0:
		start = 0
	for i in range(start,last):
		for j in range(0,w):
			if img[i,j] == 255:
				result[i,j] = 255

	return result

def Oreintation(moment):
    tmp = pow((moment['mu20'] - moment['mu02']),2)
    tmp = math.sqrt(tmp + (4*pow(moment['mu11'],2)))

    k = moment['mu02'] - moment['mu20']
    
    l = 2*moment['mu11']

    rad_maj = math.atan2(k+tmp,l)
    rad_min = math.atan2(k-tmp,l)
    return rad_maj,rad_min

def nothing(variable):
	pass

def rebalance(image):
	h, w, ch = image.shape
	m = cv2.mean(image)
	img = np.zeros((h,w),dtype=np.float)
	img = image.copy()
	b, g, r = cv2.split(img)

	gg = m[0]*1.0 / m[1]*1.0
	rr = m[0]*1.0 / m[2]*1.0

	matrix_b = np.zeros((h,w),dtype=np.float32)
	matrix_g = np.zeros((h,w),dtype=np.float32)
	matrix_r = np.zeros((h,w),dtype=np.float32)
	matrix_b.fill(1.0)
	matrix_g.fill(gg)
	matrix_r.fill(rr)

	b = b*matrix_b
	g = g*matrix_g
	r = r*matrix_r

	result = cv2.merge((b,g,r))
	result_uint = result.astype(np.uint8)

	# ____________________________ #

	img = cv2.cvtColor(result_uint,cv2.COLOR_BGR2Lab)
	L, a, b = cv2.split(img)
	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
 	L = clahe.apply(L)
 	
	result = cv2.merge((L,a,b))
	result_bgr = cv2.cvtColor(result,cv2.COLOR_Lab2BGR)
	result_hsv = cv2.cvtColor(result_bgr,cv2.COLOR_BGR2HSV)
	# h, s, v = cv2.split(result_hsv)
	# print h.max(),s.max(),v.max()
	return result_hsv

# def cut_contours(hh,ww,c):
# 	if cv2.contourArea(c):
# 		print '11'
# 		x,y,w,h = cv2.boundingRect(c)
# 		print x, x+w, y, y+h
# 		if x <= 1 or x+w >= ww :
# 			return False
# 		return True
# 	return False

def process_img_down(img):
	bgr = stretching(img.copy())
	hsv = clahe(bgr.copy())
	bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
	median = cv2.medianBlur(bgr.copy(),7)
	hsv = cv2.cvtColor(median,cv2.COLOR_BGR2HSV)	

	return hsv

def process_img_top(img):
	bgr = stretching(img.copy())
	# hsv = clahe(bgr.copy())
	hsv = cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
	return hsv