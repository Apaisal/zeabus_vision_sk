#!/usr/bin/env python
'''
Copyright @ EmOne (Thailand) Co.Ltd. 2017
Author: Anol Paisal <info@emone.co.th>
Date: 2017/05/15
'''

#import sys
import cv2
import numpy as np
#import rospkg
import rospy
#import roslib
from sensor_msgs.msg import CompressedImage, Image
#import dynamic_reconfigure.client
#from cv_bridge import CvBridge, CvBridgeError
#import time
#import math
from std_msgs.msg import String
from zeabus_vision_sk.srv import *

img = None
img_gray = None
hsv = None
client = None
wait = False
thresh = 170
lower_h = 33
lower_s = 110
lower_v = 0
upper_h = 255
upper_s = 255
upper_v = 255
lower_r = 0
lower_g = 0
lower_b = 0
upper_r = 255
upper_g = 255
upper_b = 255
gamma = 7
#bridge = CvBridge()
pub = rospy.Publisher('octopus_locator', String, queue_size=10)
    
def on_gamma_callback(param):
   global gamma
   gamma = param
   
def adjust_gamma(image, gamma=1):
# build a lookup table mapping the pixel values [0, 255] to
# their adjusted gamma values
    if gamma == 0:
        g = 1.0
    else:
        g = gamma / 10.0   
    invGamma = 1.0 / g
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")
     # apply gamma correction using the lookup table
    return cv2.LUT(image, table)
    
def on_lower_r_trackbar(param):
    global lower_r
    lower_r = param
def on_lower_g_trackbar(param):
    global lower_g
    lower_g = param
def on_lower_b_trackbar(param):
    global lower_b
    lower_b = param
def on_upper_r_trackbar(param):
    global upper_r
    upper_r = param
def on_upper_g_trackbar(param):
    global upper_g
    upper_g = param
def on_upper_b_trackbar(param):
    global upper_b
    upper_b = param
def on_lower_h_trackbar(param):
    global lower_h
    lower_h = param
def on_lower_s_trackbar(param):
    global lower_s
    lower_s = param
def on_lower_v_trackbar(param):
    global lower_v
    lower_v = param
def on_upper_h_trackbar(param):
    global upper_h
    upper_h = param
def on_upper_s_trackbar(param):
    global upper_s
    upper_s = param
def on_upper_v_trackbar(param):
    global upper_v
    upper_v = param
def on_threshold_trackbar(param):
    global thresh
    thresh = param

def process():
    global hsv, img_gray, thresh, lower_h, lower_s, lower_v, upper_h, upper_s, upper_v, lower_r, lower_g, lower_b, upper_r, upper_g, upper_b
            #blur grey 3*3 
#    img_gray = cv2.medianBlur(img_gray,5)
#        img_gray = cv2.GaussianBlur(img_gray,(5,5), 0)
#	hist,bins = np.histogram(img.ravel(),256,[0,256])
#    print thresh, lower_h,lower_s,lower_v, upper_h,upper_s,upper_v
# define range of border color in HSV lower [130,40,40] upper [255,255,255]
# define range of border color in BGR lower [0,0,0] upper [255,68,255]	
    #lower_hsv = np.array([lower_h,lower_s,lower_v])
    #upper_hsv = np.array([upper_h,upper_s,upper_v])
    #lower_bgr = np.array([lower_r,lower_g,lower_b])
    #upper_bgr = np.array([upper_r,upper_g,upper_b])

#    mask = cv2.inRange(hsv, lower, upper)
    #mask_hsv = cv2.inRange(hsv, lower_hsv, upper_hsv)
    #mask_bgr = cv2.inRange(img, lower_bgr, upper_bgr)
    # Bitwise-AND mask
#    print mask_hsv.shape, mask_bgr.shape
    #if mask_hsv.shape == mask_bgr.shape:
        #mask = cv2.bitwise_and(mask_hsv, mask_bgr)
        #res = cv2.bitwise_and(img_gray, img_gray, mask=mask)	
        #cv2.imshow('mask', res)

    #blur grey 3*3 
    #blur = cv2.medianBlur(img_gray,3)
    blur = cv2.GaussianBlur(img_gray,(7,7), 1)
    cv2.imshow('blur', blur)

#        hist,bins = np.histogram(img.ravel(),256,[0,256])
#    ret, thresh_out = cv2.threshold(img_gray, thresh, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thresh_out = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
#        thresh_out = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)
    cv2.imshow('thresh out', thresh_out)

# noise removal
    kernel = np.ones((3,3),np.uint8)
#        opening = cv2.morphologyEx(blur, cv2.MORPH_GRADIENT, kernel, iterations = 2)
    opening = cv2.morphologyEx(blur, cv2.MORPH_OPEN, kernel, iterations = 3)
#    opening = cv2.morphologyEx(blur, cv2.MORPH_CLOSE, kernel, iterations = 1)
#        opening = cv2.morphologyEx(blur, cv2.MORPH_CLOSE + cv2.MORPH_OPEN, kernel, iterations = 2)
#        opening = cv2.morphologyEx(thresh_out, cv2.MORPH_CLOSE + cv2.MORPH_OPEN, kernel, iterations = 2)
    cv2.imshow('opening', opening)
# sure background area
    sure_bg = cv2.dilate(opening, kernel, iterations=3)
       

    dist = cv2.distanceTransform(opening, cv2.DIST_L2, 3)
    cv2.normalize(dist, dist, 0, 255, cv2.NORM_MINMAX)
    dist = np.uint8(dist)
    
    #cv2.imshow('sure_bg', sure_bg)
    #cv2.imshow('dist', dist)
    ret, sure_fg = cv2.threshold(dist, dist.max()*0.7, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
 #   sure_fg = cv2.adaptiveThreshold(dist, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    #cv2.imshow('sure_fg', sure_fg)

    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg,sure_fg)
    #cv2.imshow('unknown', unknown)

    #unknown = cv2.subtract(unknown, thresh_out)
    #cv2.imshow('unknown', unknown)
#        laplacian = cv2.Laplacian(sure_bg,cv2.CV_64F)
#        cv2.imshow('laplacian', laplacian)
   # circles = cv2.HoughCircles(unknown ,cv2.HOUGH_GRADIENT,1,20,
   #                         param1=50,param2=30,minRadius=0,maxRadius=0)
    
#    im2, contours, hierarchy = cv2.findContours(dist, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
 #   im2, contours, hierarchy = cv2.findContours(unknown, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    im2, contours, hierarchy = cv2.findContours(unknown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#    hull = [] 
    minRect = []
#    minEllipse = []
#    drawing = np.zeros(thresh_out.shape, np.uint8)
#    drawing = np.zeros(dist.size, np.uint8)
    drawing = hsv.copy()
    hsv_drawing = hsv.copy()
#   for i in range(len(contours)):
#	hull.append(cv2.convexHull(contours[i]))
    ret = []
    for i in range(len(contours)):
        minRect.append(cv2.minAreaRect(contours[i]))
#        if len(contours[i]) < 5:
#       	    minRect.append(cv2.minAreaRect(contours[i]))
#            minEllipse.append(cv2.fitEllipse(contours[i]))
#            cv2.ellipse(drawing, minEllipse[i], (255,0,0), 2)
#            cv2.drawContours(drawing, contours, int(i), (0,255,0), 0) 
        
    cv2.drawContours(drawing, contours, -1, (0,0,255), cv2.FILLED) 
    cv2.imshow('Contour', drawing)
    for i in range(len(contours)):
#	if contours[i].size > 100:
#        cv2.drawContours(drawing, contours, i, (0,255,0), 1) 
#        cv2.drawContours(drawing, hull, i, (0,255,0), 0) 
	
        box = cv2.boxPoints(minRect[i])
        box = np.int0(box)
        if box[1][1] < box[3][1] and box[0][0] < box[2][0]:
            img_roi = hsv[int(box[1][1]):int(box[3][1]), int(box[0][0]):int(box[2][0])]
            img_roi = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
            #cv2.imshow('ROI', img_roi)
            # create a CLAHE object (Arguments are optional).
            clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(3,3))
            cl1 = clahe.apply(img_roi)
            res = np.hstack((img_roi, cl1))
            
            #equ = cv2.equalizeHist(img_roi)
            #res = np.hstack((img_roi, img_roi))
            cv2.imshow('equ', res)
#            lower_blue = np.array([110,50,50])
#            upper_blue = np.array([130,255,255])
#            mask = cv2.inRange(img_roi, lower_blue, upper_blue)
#            res = cv2.bitwise_and(img_roi,img_roi, mask= mask)
            circles = cv2.HoughCircles(img_roi, cv2.HOUGH_GRADIENT, 1.5, 100, param1=80, param2=20, minRadius=0, maxRadius=0)
            if circles != None:
                for i in circles[0,:]:
            # draw the outer circle
                    #cv2.imshow('ROI', img_roi)
                    cv2.circle(hsv_drawing,(int(box[1][0]+i[0]),int(box[1][1]+i[1])),i[2],(0,255,0),2)
            # draw the center of the circle
#                    cv2.circle(hsv,(i[0],i[1]),2,(0,0,255),3)
                    cv2.circle(hsv_drawing,(int(box[1][0]+i[0]),int(box[1][1]+i[1])),2,(0,0,255),3)
                    cv2.imshow('circle', hsv_drawing)
                    #return (int(box[1][0]+i[0]), int(box[1][1]+i[1]), int(i[2]))
#                    cv2.imshow('circle', hsv)
#                    cv2.imshow('ROI', img_roi)
                    ret.append({"width":img_gray.shape[1], "height":img_gray.shape[0], "origin_x":int(box[1][0]+i[0]), \
                        "origin_y":int(box[1][1]+i[1]), "radius":int(i[2])})
                    #buoy = "window_x={7},window_y={8},origin_x={0},origin_y={1},radius={2},prob_blue={3},prob_green={4},prob_red={5},color={6}" \
                       #             .format(ret[i][0], ret[i][1], ret[i][2], ret[i][3], ret[i][4], ret[i][5], ret[i][6],  img_gray.shape[1], img_gray.shape[0])
    print("Count all: {}".format(len(ret)))
    rospy.loginfo(ret)
    return ret

#        cv2.drawContours(drawing, [box], 0, (255,255,255), cv2.FILLED) 

#	for j in range(4):
#	    cv2.line(drawing, box[j], box[(j+1)%4], (0,255,0), 1)
#    img_unknown = cv2.cvtColor(drawing, cv2.COLOR_GRAY2BGR)
#    cv2.imshow('img_unknown', img_unknown)

#    im2, contours, hierarchy = cv2.findContours(drawing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#    cv2.watershed(img, img_unknown)
#    cv2.imshow('img_unknown', img_unknown)

#    cv2.namedWindow('Contour', flags=cv2.WINDOW_NORMAL)
#    cv2.imshow('Contour', drawing)

def callback(msg):
#    global img, img_gray, hsv
#    if wait == False:
#        try:
#            img_orig = bridge.imgmsg_to_cv2(msg, "bgr8")
#        except CvBridgeError as e:
#            print(e) 
#        arr = np.fromstring(msg.data, np.uint8)
#        print len(arr)
#        img_orig = cv2.imdecode(arr, cv2.IMREAD_COLOR)
#        print img_orig
#        if img_orig == None:
#            im_orig = np.ones((320,256),np.uint8)
#            return
#        (height, width, ch) = img_orig.shape
#        print (height, width, ch)
#        img = cv2.resize(img_orig, (320, 180))
#        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)     
    global img, img_gray, hsv, gamma
    if wait == False:
        arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(arr, 1)

def getchar():
   #Returns a single character from standard input
   import tty, termios, sys
   fd = sys.stdin.fileno()
   old_settings = termios.tcgetattr(fd)
   try:
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
   finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
   return ch
           
def main():
    global client, img, img_gray, thresh,  hsv,  pub
    rate = rospy.Rate(10) # 10Hz
    
    cv2.namedWindow('Source', flags=cv2.WINDOW_NORMAL)
    cv2.createTrackbar('Threshold: ', 'Source', thresh, 255, on_threshold_trackbar)
    cv2.createTrackbar('Gamma: ', 'Source', gamma, 50, on_gamma_callback)

#    cv2.createTrackbar('lower h: ', 'Source', lower_h, 255, on_lower_h_trackbar)
#    cv2.createTrackbar('lower s: ', 'Source', lower_s, 255, on_lower_s_trackbar)
#    cv2.createTrackbar('lower v: ', 'Source', lower_v, 255, on_lower_v_trackbar)
#    cv2.createTrackbar('upper h: ', 'Source', upper_h, 255, on_upper_h_trackbar)
#    cv2.createTrackbar('upper s: ', 'Source', upper_s, 255, on_upper_s_trackbar)
#    cv2.createTrackbar('upper v: ', 'Source', upper_v, 255, on_upper_v_trackbar)
#    cv2.createTrackbar('lower r: ', 'Source', lower_r, 255, on_lower_r_trackbar)
#    cv2.createTrackbar('lower g: ', 'Source', lower_g, 255, on_lower_g_trackbar)
#    cv2.createTrackbar('lower b: ', 'Source', lower_b, 255, on_lower_b_trackbar)
#    cv2.createTrackbar('upper r: ', 'Source', upper_r, 255, on_upper_r_trackbar)
#    cv2.createTrackbar('upper g: ', 'Source', upper_g, 255, on_upper_g_trackbar)
#    cv2.createTrackbar('upper b: ', 'Source', upper_b, 255, on_upper_b_trackbar)

    while(img is None):
        rate.sleep()
    
    while not rospy.is_shutdown():
        img_resize = cv2.resize(img, (img.shape[1] / 4, img.shape[0]/4))
        img_gamma = adjust_gamma(img_resize, gamma)
        hsv = cv2.cvtColor(img_gamma, cv2.COLOR_BGR2HSV)
        img_gray = cv2.cvtColor(img_gamma, cv2.COLOR_BGR2GRAY)
        cv2.imshow('Source', img_gamma)
        
        h, s, v = cv2.split(hsv)
        h_inv = cv2.bitwise_not(h) 
        ret, mask = cv2.threshold(h_inv, thresh, 255, cv2.THRESH_BINARY)
        
        sure_fg = cv2.bitwise_and(img_gamma, img_gamma,  mask=mask)
        img_gray = cv2.cvtColor(sure_fg, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('img_gray', img_gray )
        
        #if old_frame == None:
          #  old_frame = img_gamma.copy()
          #  pass
        #else :
        res = process()
        if res != None:
           pub.publish(res)
 
        #old_frame = img_gamma.copy()
#        if img_gray == None:
#           cv2.destroyAllWindows()
       # cv2.imshow('Source', img_resize)
#        cv2.imshow('Source', img)       	
#        process() 

        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            break
        rate.sleep()

    cv2.destroyAllWindows()
#    exit()
    
if __name__ == '__main__':
    try:
        rospy.init_node('TPD', anonymous=True)
        topic = "/leftcam_top/image_raw/compressed"
#        topic = "camera/image_raw"
        rospy.Subscriber(topic, CompressedImage, callback)
#        rospy.Subscriber(topic, Image, callback)
        main()
    except rospy.ROSInterruptException:
        pass
