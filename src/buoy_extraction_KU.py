#!/usr/bin/env python
"""
Copyright @ EmOne (Thailand) Co.Ltd. 2017
Author: Anol Paisal <info@emone.co.th>
Date: 2017/05/15
"""
import cv2
import numpy as np
#import rospkg
import rospy
from sensor_msgs.msg import CompressedImage
#import dynamic_reconfigure.client
#import time
import math
#from matplotlib import pyplot as plt
from std_msgs.msg import String

img = None
img_gamma = None
img_gray = None
img_resize = None
hsv = None
old_frame = None
client = None
wait = False
thresh = 184
gamma = 30
color = None

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

def on_threshold_callback(param):
   global thresh
   thresh = param

def threshold_callback(params):
    global img_gray, thresh, gamma, img_gamma
    thresh = params
#blur grey 3*3 
    #kernel = np.ones((5,5),np.float32)/25
    #img_gray = cv2.filter2D(img_gray, -1, kernel)
    #img_gray = cv2.bilateralFilter(img_gray,9,75,75)
    #img_gray = cv2.medianBlur(img_gray, 3)
    img_gray = cv2.GaussianBlur(img_gray,(7,7), 0)
    cv2.imshow('blur', img_gray)

    ret, thresh_out = cv2.threshold(img_gray, thresh, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
#    img_gray = adjust_gamma(img_gray, gamma)
#    cv2.imshow('gamma', img_gray)
    #thresh_out = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
#    thresh_out = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)
   # cv2.imshow('thresh out', thresh_out)

# noise removal
    kernel = np.ones((3,3),np.uint8)
    #opening = cv2.morphologyEx(img_gray, cv2.MORPH_GRADIENT, kernel, iterations = 2)
    opening = cv2.morphologyEx(thresh_out, cv2.MORPH_OPEN, kernel, iterations = 2)
    #opening = cv2.morphologyEx(thresh_out, cv2.MORPH_CLOSE, kernel, iterations = 2)
    #opening = cv2.morphologyEx(thresh_out, cv2.MORPH_CLOSE + cv2.MORPH_OPEN, kernel, iterations = 2)
    cv2.imshow('opening', opening)
# sure background area
    sure_bg = cv2.dilate(opening, kernel, iterations=5)
    cv2.imshow('sure_bg', sure_bg)

    dist = cv2.distanceTransform(opening, cv2.DIST_L2, 3)
    cv2.normalize(dist, dist, 0, 255, cv2.NORM_MINMAX)
    dist = np.uint8(dist)

    cv2.imshow('dist', dist)
   # ret, sure_fg = cv2.threshold(dist, dist.max()*0.7, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    sure_fg = cv2.adaptiveThreshold(dist, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    cv2.imshow('sure_fg', sure_fg)

    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg,sure_fg)
    cv2.imshow('unknown', unknown)

    #unknown = cv2.subtract(unknown, thresh_out)
    #cv2.imshow('unknown', unknown)
   # laplacian = cv2.Laplacian(unknown,cv2.CV_64F)
    #cv2.imshow('laplacian', laplacian)
   # circles = cv2.HoughCircles(unknown ,cv2.HOUGH_GRADIENT,1,20,
   #                         param1=50,param2=30,minRadius=0,maxRadius=0)
    
    #im2, contours, hierarchy = cv2.findContours(dist, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#    im2, contours, hierarchy = cv2.findContours(unknown, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
#    im2, contours, hierarchy = cv2.findContours(unknown, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   # im2, contours, hierarchy = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    im2, contours, hierarchy = cv2.findContours(unknown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#    hull = [] 
    minRect = []
#    minEllipse = []
    hsv_drawing = hsv.copy()
    drawing = hsv.copy()
    #drawing =  np.zeros(unknown.shape, np.uint8)
#    drawing = np.zeros(dist.size, np.uint8)
    print(len(contours))
#    for i in range(len(contours)):
#	hull.append(cv2.convexHull(contours[i]))
    
    for i in range(len(contours)):
        minRect.append(cv2.minAreaRect(contours[i]))
        x,y,w,h = cv2.boundingRect(contours[i])
        cv2.rectangle(drawing,(x,y),(x+w,y+h),(0,0, 255),1)
       # if len(contours[i]) < 5:
#       	    minRect.append(cv2.minAreaRect(contours[i]))
#            minEllipse.append(cv2.fitEllipse(contours[i]))
#            cv2.ellipse(drawing, minEllipse[i], (255,0,0), 2)
            #cv2.drawContours(drawing, contours, int(i), (255,255,255), 0) 
        
        cv2.drawContours(drawing, contours, i, (0, 255, 0), 1) 
        #cv2.drawContours(drawing, contours, -1, (255,255,255), cv2.FILLED) 
        cv2.imshow('Contour', drawing)
    for i in range(len(contours)):
#	if contours[i].size > 100:
#        cv2.drawContours(drawing, contours, i, (0,255,0), 1) 
#        cv2.drawContours(drawing, hull, i, (0,255,0), 0) 
	
        box = cv2.boxPoints(minRect[i])
        box = np.int0(box)
        if box[1][1] < box[3][1] and box[0][0] < box[2][0]:
            img_roi = hsv[int(box[1][1]):int(box[3][1]), int(box[0][0]):int(box[2][0])]
            
            img_rgb_roi = cv2.cvtColor(img_roi, cv2.COLOR_HSV2BGR)
            
            img_roi = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
            cv2.imshow('ROI', img_roi)
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
  #          circles = cv2.HoughCircles(cl1, cv2.HOUGH_GRADIENT, 1, 1, param1=80, param2=20, minRadius=0, maxRadius=0)
            circles = cv2.HoughCircles(img_roi, cv2.HOUGH_GRADIENT, 1, 1, param1=80, param2=20, minRadius=0, maxRadius=0)
            if circles != None:
                for i in circles[0,:]:
            # draw the outer circle
                    
                    #cv2.imshow('ROI', img_roi)
                    cv2.circle(hsv_drawing,(int(box[1][0]+i[0]),int(box[1][1]+i[1])),i[2],(0,255,0),2)
            # draw the center of the circle
#                    cv2.circle(hsv,(i[0],i[1]),2,(0,0,255),3)
                    cv2.circle(hsv_drawing,(int(box[1][0]+i[0]),int(box[1][1]+i[1])),2,(0,0,255),3)
                    cv2.imshow('circle', hsv_drawing)
#                   cv2.imshow('img_rgb_roi', img_rgb_roi)
                 
                    # create a water index pixel mask
                    w = img_rgb_roi[0,0]
#                    print w
                    b,g,r =cv2.split(img_rgb_roi)
                    mask = np.zeros(img_rgb_roi.shape[:2], np.uint8)

                    mask[:, :] = w[0]  
                    #mask_inv = cv2.bitwise_not(mask)  
#                    cv2.imshow('b_mask',mask)                
                    b_masked_img = cv2.subtract(b, mask)
#                    cv2.imshow('b_masked_img', b_masked_img)
                    b_histr,bins = np.histogram(b_masked_img.ravel(), 256,[0,256])
                    b_cdf = b_histr.cumsum()
#                    b_cdf_normalized = b_cdf * b_histr.max()/ b_cdf.max()
                   
                    mask[:, :] = w[1]
                    #mask_inv = cv2.bitwise_not(mask)
                    #cv2.imshow('g_mask',mask)                
                    g_masked_img = cv2.subtract(g, mask)
#                    cv2.imshow('g_masked_img', g_masked_img)
                    g_histr,bins = np.histogram(g_masked_img.ravel(), 256,[0,256])
                    g_cdf = g_histr.cumsum()
#                    g_cdf_normalized = g_cdf * g_histr.max()/ g_cdf.max()
                    
                    mask[:, :] = w[2]               
                    #mask_inv = cv2.bitwise_not(mask)
                    #cv2.imshow('r_mask',mask)                     
                    r_masked_img = cv2.subtract(r, mask)
#                    cv2.imshow('r_masked_img', r_masked_img)
                    r_histr,bins = np.histogram(r_masked_img.ravel(), 256,[0,256])
                    r_cdf = r_histr.cumsum()
#                    r_cdf_normalized = r_cdf * r_histr.max()/ r_cdf.max()
                    
#                    m_img = cv2.merge((b_masked_img,g_masked_img,r_masked_img))
#                    cv2.imshow('m_img', m_img)
#                    b_histr = cv2.calcHist([img_rgb_roi],[0],None,[256],[0,256])                   
#                    g_histr = cv2.calcHist([img_rgb_roi],[1],None,[256],[0,256])                   
#                    r_histr = cv2.calcHist([img_rgb_roi],[2],None,[256],[0,256])                   
                    total = (b_cdf.max() - b_histr[0]) + (g_cdf.max() - g_histr[0]) + (r_cdf.max() - r_histr[0])
                    if total == 0:
                        Pb = 1.0
                        Pg = 1.0
                        Pr = 1.0
                    else   :
                        Pb = (float(b_cdf.max() - b_histr[0]) / float(total))
                        Pg = (float(g_cdf.max() - g_histr[0]) / float(total))
                        Pr = (float(r_cdf.max() - r_histr[0]) / float(total))
                        
                    if abs(Pg-Pr) < 0.2:
                        color = 'y'
                    elif Pg > Pb and Pg > Pr:
                        color = 'g'
                    elif Pr > Pb and Pr > Pg:
                        color = 'r'
                        
                    #print "Pb{}Pg{}Pr{}".format(Pb, Pg, Pr)
#                    hist,bins = np.histogram(img_roi.flatten(), 256,[0,256])
#                    cdf = hist.cumsum()
#                    cdf_normalized = cdf * hist.max()/ cdf.max()
#                    plt.plot(cdf_normalized, color = 'b')
#                    plt.hist(img.flatqten(),256,[0,256], color = 'r')
#                    plt.xlim([0,256])
#                    plt.legend(('cdf','histogram'), loc = 'upper left')
#                    plt.show()



#                    return (int(box[1][0]+i[0]), int(box[1][1]+i[1]), i[2], b_histr.max(), #g_histr.max(), r_histr.max())
                    return (int(box[1][0]+i[0]), int(box[1][1]+i[1]), int(i[2]), Pb, Pg, Pr,  color)

#        cv2.drawContours(drawing, [box], 0, (255,255,255), cv2.FILLED) 

#	for j in range(4):
#	    cv2.line(drawing, box[j], box[(j+1)%4], (0,255,0), 1)
#    img_unknown = cv2.cvtColor(drawing, cv2.COLOR_GRAY2BGR)
#    cv2.imshow('img_unknown', img_unknown)

#    im2, contours, hierarchy = cv2.findContours(drawing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#    cv2.watershed(img, img_unknown)
#    cv2.imshow('img_unknown', img_unknown)

#    cv2.namedWindow('Contour', flags=cv2.WINDOW_NORMAL)
    #cv2.imshow('Contour', drawing)

def callback(msg):
    global img, img_gray, hsv, gamma,  img_resize
    if wait == False:
        arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(arr, 1)

def main():
    global client, img, img_gray, thresh, img_gamma,  hsv,  img_resize,  old_frame

    pub = rospy.Publisher('buoy_locator', String, queue_size=100)
    rate = rospy.Rate(10) # 10Hz
    
    cv2.namedWindow('Source', flags=cv2.WINDOW_NORMAL)
    cv2.createTrackbar('Threshold: ', 'Source', thresh, 255, on_threshold_callback)
    cv2.createTrackbar('Gamma: ', 'Source', gamma, 50, on_gamma_callback)
    
    while(img is None):
        rate.sleep() #rospy.sleep(0.01)
    
    while not rospy.is_shutdown():
        img_resize = cv2.resize(img, (img.shape[1]/4, img.shape[0]/4))
        img_gamma = adjust_gamma(img_resize, gamma)
        hsv = cv2.cvtColor(img_gamma, cv2.COLOR_BGR2HSV)
        img_gray = cv2.cvtColor(img_gamma, cv2.COLOR_BGR2GRAY)
        cv2.imshow('Source', img_gamma)
    
        
        #cv2.imshow('hsv', hsv)
        h, s, v = cv2.split(hsv)
        h_inv = cv2.bitwise_not(h)
        ret, mask1 = cv2.threshold(h_inv, 162, 255, cv2.THRESH_BINARY_INV)
        mask1_inv = cv2.bitwise_not(mask1)
        #cv2.imshow('mask_inv', mask1_inv)
        ret, mask2 = cv2.threshold(h_inv, 150, 255, cv2.THRESH_BINARY)
        mask2_inv = cv2.bitwise_not(mask2)
        #cv2.imshow('mask2_inv', mask2_inv)
        mask  = cv2.bitwise_or(mask1_inv,  mask2_inv)
        #cv2.imshow('mask', mask)
        img_gray = cv2.bitwise_and(img_gray,img_gray,mask = mask)
        #cv2.imshow('Gray', img_gray )
  	
        if old_frame == None:
            old_frame = img_gamma.copy()
            pass
        else :
            res = threshold_callback(thresh) 
            if res != None:
               buoy = "window_x={7},window_y={8},origin_x={0},origin_y={1},radius={2},prob_blue={3},prob_green={4},prob_red={5},color={6}".format(res[0], res[1], res[2], res[3], res[4], res[5], res[6],  img_gray.shape[1], img_gray.shape[0])
               rospy.loginfo(buoy)
               pub.publish(buoy)
 
        old_frame = img_gamma.copy()
        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            break
        rate.sleep()
        
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        rospy.init_node('buoy', anonymous=True)
#        topic = "/image_raw/compressed"
        topic = "/top/center/image_rect_color/compressed"
        rospy.Subscriber(topic, CompressedImage, callback)
        main()
    except rospy.ROSInterruptException:
        pass
    
