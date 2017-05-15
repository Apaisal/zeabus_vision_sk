#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage
#import dynamic_reconfigure.client
import time
import math

img = None
img_gray = None
hsv = None
client = None
wait = False
thresh = 100

def threshold_callback(params):
    global img_gray, thresh
    thresh = params
#    ret, thresh_out = cv2.threshold(img_gray, thresh, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thresh_out = cv2.adaptiveThreshold(img_gray, 255, \
    		cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
#    thresh_out = cv2.adaptiveThreshold(img_gray, 255, \
#		cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)
#    cv2.imshow('thresh out', thresh_out)

# noise removal
    kernel = np.ones((3,3),np.uint8)
    opening = cv2.morphologyEx(thresh_out, cv2.MORPH_GRADIENT, kernel, iterations = 5)
    #opening = cv2.morphologyEx(thresh_out, cv2.MORPH_OPEN, kernel, iterations = 2)
    #opening = cv2.morphologyEx(thresh_out, cv2.MORPH_CLOSE, kernel, iterations = 2)
    #opening = cv2.morphologyEx(thresh_out, cv2.MORPH_CLOSE + cv2.MORPH_OPEN, kernel, iterations = 2)
#    cv2.imshow('opening', opening)
# sure background area
    sure_bg = cv2.dilate(opening, kernel, iterations=5)
#    cv2.imshow('sure_bg', sure_bg)

    dist = cv2.distanceTransform(opening, cv2.DIST_L2, 3)
    cv2.normalize(dist, dist, 0, 255, cv2.NORM_MINMAX)
    dist = np.uint8(dist)

#    cv2.imshow('dist', dist)
#    ret, sure_fg = cv2.threshold(dist, dist.max()*0.7, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    sure_fg = cv2.adaptiveThreshold(dist, 255, \
    		cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
#    cv2.imshow('sure_fg', sure_fg)

    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg,sure_fg)
#    cv2.imshow('unknown', unknown)

    #unknown = cv2.subtract(unknown, thresh_out)
    #cv2.imshow('unknown', unknown)
#    laplacian = cv2.Laplacian(unknown,cv2.CV_64F)
#    cv2.imshow('laplacian', laplacian)
   # circles = cv2.HoughCircles(unknown ,cv2.HOUGH_GRADIENT,1,20,
   #                         param1=50,param2=30,minRadius=0,maxRadius=0)
    
#    im2, contours, hierarchy = cv2.findContours(dist, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#    im2, contours, hierarchy = cv2.findContours(unknown, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    im2, contours, hierarchy = cv2.findContours(unknown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#    hull = [] 
    minRect = []
#    minEllipse = []
    drawing = np.zeros(thresh_out.shape, np.uint8)
#    drawing = np.zeros(dist.size, np.uint8)

#    for i in range(len(contours)):
#	hull.append(cv2.convexHull(contours[i]))

    for i in range(len(contours)):
        minRect.append(cv2.minAreaRect(contours[i]))
#        if len(contours[i]) < 5:
#       	    minRect.append(cv2.minAreaRect(contours[i]))
#            minEllipse.append(cv2.fitEllipse(contours[i]))
#            cv2.ellipse(drawing, minEllipse[i], (255,0,0), 2)
#            cv2.drawContours(drawing, contours, int(i), (0,255,0), 0) 
        
#    cv2.drawContours(drawing, contours, 0, (0,0,255), 1) 
    for i in range(len(contours)):
#	if contours[i].size > 100:
#        cv2.drawContours(drawing, contours, i, (0,255,0), 1) 
#        cv2.drawContours(drawing, hull, i, (0,255,0), 0) 
	
        box = cv2.boxPoints(minRect[i])
        box = np.int0(box)
	if box[1][1] < box[3][1] and box[0][0] < box[2][0]:
            img_roi = hsv[int(box[1][1]):int(box[3][1]), int(box[0][0]):int(box[2][0])]
            img_roi = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
            cv2.imshow('ROI', img_roi)
            
#            lower_blue = np.array([110,50,50])
#            upper_blue = np.array([130,255,255])
#            mask = cv2.inRange(img_roi, lower_blue, upper_blue)
#            res = cv2.bitwise_and(img_roi,img_roi, mask= mask)
	    circles = cv2.HoughCircles(img_roi, cv2.HOUGH_GRADIENT, 1, 10, param1=80, param2=25, minRadius=0, maxRadius=0)
            if circles != None:
                for i in circles[0,:]:
            # draw the outer circle
                    cv2.circle(hsv,(int(box[1][0]+i[0]),int(box[1][1]+i[1])),i[2],(0,255,0),2)
            # draw the center of the circle
#                    cv2.circle(hsv,(i[0],i[1]),2,(0,0,255),3)
                    cv2.circle(hsv,(int(box[1][0]+i[0]),int(box[1][1]+i[1])),2,(0,0,255),3)
                    cv2.imshow('circle', hsv)
#                    cv2.imshow('ROI', img_roi)

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
    global img, img_gray, hsv
    if wait == False:
        arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(arr, 1)
        img = cv2.resize(img, (320, 256))
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

def main():
    global client, img, img_gray, thresh
    cv2.namedWindow('Source', flags=cv2.WINDOW_NORMAL)
    cv2.createTrackbar('Threshold: ', 'Source', thresh, 255, threshold_callback)
    
    while(img is None):
        rospy.sleep(0.01)
    
    while True:

        #blur grey 3*3 
        img_gray = cv2.medianBlur(img_gray,5)
#        img_gray = cv2.GaussianBlur(img_gray,(5,5), 0)
	hist,bins = np.histogram(img.ravel(),256,[0,256])
        
        cv2.imshow('Source', img)
#        cv2.imshow('Gray Blur', img_gray)
  	
        threshold_callback(thresh) 

        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('AEB', anonymous=True)
    topic = "/image_raw/compressed"
    rospy.Subscriber(topic, CompressedImage, callback)
    main()
