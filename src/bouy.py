#!/usr/bin/env python
"""
Copyright @ EmOne (Thailand) Co.Ltd. 2017
Author: Anol Paisal <info@emone.co.th>
Date: 2017/05/15
"""
import sys
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
import math
sys.path.append(
    '/home/zeabus/catkin_ws/src/src_code/zeabus_vision/zeabus_vision_main/src/')
from vision_lib import *
from std_msgs.msg import String
from zeabus_vision_srv_msg.msg import vision_msg_bouy
from zeabus_vision_srv_msg.srv import vision_srv_bouy
img = None
img_gamma = None
img_gray = None
img_resize = None
hsv = None
old_frame = None
wait = False
thresh = 184
gamma = 30
color = None
width = 480
height = 300
reqColor = None


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
    global img_gray, thresh, gamma, img_gamma, width, height
    thresh = params
    img_gray = cv2.GaussianBlur(img_gray, (7, 7), 0)
    # cv2.imshow('blur', img_gray)

    ret, thresh_out = cv2.threshold(
        img_gray, thresh, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(
        thresh_out, cv2.MORPH_OPEN, kernel, iterations=2)
    # cv2.imshow('opening', opening)

    sure_bg = cv2.dilate(opening, kernel, iterations=5)
    # cv2.imshow('sure_bg', sure_bg)

    dist = cv2.distanceTransform(opening, cv2.DIST_L2, 3)
    cv2.normalize(dist, dist, 0, 255, cv2.NORM_MINMAX)
    dist = np.uint8(dist)

    # cv2.imshow('dist', dist)
    # ret, sure_fg = cv2.threshold(dist, dist.max()*0.7, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    sure_fg = cv2.adaptiveThreshold(
        dist, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    # cv2.imshow('sure_fg', sure_fg)

    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg, sure_fg)
    # cv2.imshow('unknown', unknown)

    im2, contours, hierarchy = cv2.findContours(
        unknown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#    hull = []
    minRect = []
#    minEllipse = []
    hsv_drawing = hsv.copy()
    drawing = hsv.copy()
    #drawing =  np.zeros(unknown.shape, np.uint8)
#    drawing = np.zeros(dist.size, np.uint8)
    # print(len(contours))
#    for i in range(len(contours)):
#	hull.append(cv2.convexHull(contours[i]))
    ret = []
    for i in range(len(contours)):
        minRect.append(cv2.minAreaRect(contours[i]))
        x, y, w, h = cv2.boundingRect(contours[i])
        cv2.rectangle(drawing, (x, y), (x + w, y + h), (0, 0, 255), 1)

        cv2.drawContours(drawing, contours, i, (0, 255, 0), 1)

        # cv2.imshow('Contour', drawing)
        publish_result(drawing, 'bgr', '/Contour')
    for i in range(len(contours)):

        box = cv2.boxPoints(minRect[i])
        box = np.int0(box)
        if box[1][1] < box[3][1] and box[0][0] < box[2][0]:
            img_roi = hsv[int(box[1][1]):int(box[3][1]),
                          int(box[0][0]):int(box[2][0])]

            img_rgb_roi = cv2.cvtColor(img_roi, cv2.COLOR_HSV2BGR)

            img_roi = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
            # cv2.imshow('ROI', img_roi)
            # create a CLAHE object (Arguments are optional).
            clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(3, 3))
            cl1 = clahe.apply(img_roi)
            res = np.hstack((img_roi, cl1))

            # cv2.imshow('equ', res)
#
            # circles = cv2.HoughCircles(
            # img_roi, cv2.HOUGH_GRADIENT, 1.3, 100, param1=80, param2=20,
            # minRadius=0, maxRadius=0)
            circles = cv2.HoughCircles(
                img_roi, cv2.HOUGH_GRADIENT, 2, 100, param1=80, param2=20, minRadius=0, maxRadius=50)
            if circles != None:
                for i in circles[0, :]:

                    cv2.circle(hsv_drawing, (int(
                        box[1][0] + i[0]), int(box[1][1] + i[1])), i[2], (0, 255, 0), 2)

                    cv2.circle(
                        hsv_drawing, (int(box[1][0] + i[0]), int(box[1][1] + i[1])), 2, (0, 0, 255), 3)
                    # cv2.imshow('circle', hsv_drawing)
                    publish_result(hsv_drawing, 'bgr', '/circle')
                    # create a water index pixel mask
                    w = img_rgb_roi[0, 0]
#                    print w
                    b, g, r = cv2.split(img_rgb_roi)
                    mask = np.zeros(img_rgb_roi.shape[:2], np.uint8)

                    mask[:, :] = w[0]
                    #mask_inv = cv2.bitwise_not(mask)
#                    cv2.imshow('b_mask',mask)
                    b_masked_img = cv2.subtract(b, mask)
#                    cv2.imshow('b_masked_img', b_masked_img)
                    b_histr, bins = np.histogram(
                        b_masked_img.ravel(), 256, [0, 256])
                    b_cdf = b_histr.cumsum()
#                    b_cdf_normalized = b_cdf * b_histr.max()/ b_cdf.max()

                    mask[:, :] = w[1]
                    #mask_inv = cv2.bitwise_not(mask)
                    # cv2.imshow('g_mask',mask)
                    g_masked_img = cv2.subtract(g, mask)
#                    cv2.imshow('g_masked_img', g_masked_img)
                    g_histr, bins = np.histogram(
                        g_masked_img.ravel(), 256, [0, 256])
                    g_cdf = g_histr.cumsum()
#                    g_cdf_normalized = g_cdf * g_histr.max()/ g_cdf.max()

                    mask[:, :] = w[2]
                    #mask_inv = cv2.bitwise_not(mask)
                    # cv2.imshow('r_mask',mask)
                    r_masked_img = cv2.subtract(r, mask)
#                    cv2.imshow('r_masked_img', r_masked_img)
                    r_histr, bins = np.histogram(
                        r_masked_img.ravel(), 256, [0, 256])
                    r_cdf = r_histr.cumsum()
#                    r_cdf_normalized = r_cdf * r_histr.max()/ r_cdf.max()

                    total = (
                        b_cdf.max() - b_histr[0]) + (g_cdf.max() - g_histr[0]) + (r_cdf.max() - r_histr[0])
                    if total == 0:
                        Py = 1.0
                        Pg = 1.0
                        Pr = 1.0
                    else:
                        Py = (float(b_cdf.max() - b_histr[0]) / float(total))
                        Pg = (float(g_cdf.max() - g_histr[0]) / float(total))
                        Pr = (float(r_cdf.max() - r_histr[0]) / float(total))

                    if abs(Pg - Pr) < 0.2:
                        color = 'y'
                    elif Pg > Py and Pg > Pr:
                        color = 'g'
                    elif Pr > Py and Pr > Pg:
                        color = 'r'

                    # ret.append({"width": img_gray.shape[1], "height": img_gray.shape[0], "origin_x": int(box[1][0] + i[0]),
                        # "origin_y": int(box[1][1] + i[1]), "radius": int(i[2]), "prob_blue": Py, "prob_green": Pg, "prob_red": Pr,  "color": color})
                    # color, prob(r,y,g), r, x, y
                    ret.append([color, Pr, Py, Pg, int(i[2]), int(
                        box[1][0] + i[0]), int(box[1][1] + i[1])])
    print("Count all: {}".format(len(ret)))
    rospy.loginfo(ret)
    return ret


def callback(msg):
    global img, img_gray, hsv, gamma, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(
        arr, 1), (width, height))


def mission_callback(msg):
    global task, req, reqColor
    task = msg.task.data
    req = msg.req.data
    reqColor = req
    return find_bouy()


def find_bouy():
    global client, img, img_gray, thresh, img_gamma,  hsv,  img_resize,  reqColor
    res = None
    # cv2.namedWindow('Source', flags=cv2.WINDOW_NORMAL)
    # cv2.createTrackbar('Threshold: ', 'Source', thresh,
    #                    255, on_threshold_callback)
    # cv2.createTrackbar('Gamma: ', 'Source', gamma, 50, on_gamma_callback)
    m = vision_msg_bouy()
    while img is None:
        print('img none')
        rospy.sleep(0.1)  # rospy.sleep(0.01)
    img_gamma = adjust_gamma(img, gamma)
    hsv = cv2.cvtColor(img_gamma, cv2.COLOR_BGR2HSV)
    img_gray = cv2.cvtColor(img_gamma, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('Source', img_gamma)

    h, s, v = cv2.split(hsv)
    h_inv = cv2.bitwise_not(h)
    ret, mask1 = cv2.threshold(h_inv, 162, 255, cv2.THRESH_BINARY_INV)
    mask1_inv = cv2.bitwise_not(mask1)
    ret, mask2 = cv2.threshold(h_inv, 150, 255, cv2.THRESH_BINARY)
    mask2_inv = cv2.bitwise_not(mask2)
    #cv2.imshow('mask2_inv', mask2_inv)
    mask = cv2.bitwise_or(mask1_inv,  mask2_inv)
    #cv2.imshow('mask', mask)
    img_gray = cv2.bitwise_and(img_gray, img_gray, mask=mask)
    # cv2.imshow('Gray', img_gray)

    # if old_frame == None:
    #     old_frame = img_gamma.copy()
    #     pass
    # else:
    offset_c = width / 2.0
    offset_r = height / 2.0
    result = threshold_callback(thresh)
    result_dict = {'r': [], 'g': [], 'y': [], 'a': []}
    if len(result) > 0:
        for res in result:
            # color, prob(r,y,g), r, x, y
            [color, Pr, Py, Pg, r, x, y] = res
            resmsg = [color, r, math.pi * r * r, x, y]
            # if r
            if len(result_dict[color]) <= 0:
                result_dict[color].append(resmsg)
            else:
                if result_dict[color][0][1] < resmsg[1]:
                    result_dict[color].pop()
                    result_dict[color].append(resmsg)
            cv2.circle(img_gray, (int(x), int(y)), 3, (0, 255, 255), -1)

    else:
        m.cx = [0]
        m.cy = [0]
        m.area = [0]
        m.prob = [0]
        m.num = 0
        m.color = [0]
        m.appear = False
        return m
# /////////////////////////////////////////////////////////////////////////////////////////////////
    if len(result_dict[reqColor]) <= 0 and not reqColor == 'a':
        m.cx = [0]
        m.cy = [0]
        m.area = [0]
        m.prob = [0]
        m.num = 0
        m.color.append(0)
        m.appear = False
        return m
    elif reqColor == 'a':
        m.num = 0
        if len(result_dict['r']) > 0:
            color, r, area, x, y = result_dict['r'][0]
            m.cx.append(x)
            m.cy.append(y)
            m.area.append(area)
            m.prob.append(0)
            m.num += 1
            m.color.append(1)
        if len(result_dict['y']) > 0:
            color, r, area, x, y = result_dict['y'][0]
            m.cx.append(x)
            m.cy.append(y)
            m.area.append(area)
            m.prob.append(0)
            m.num += 1
            m.color.append(2)
        if len(result_dict['g']) > 0:
            color, r, area, x, y = result_dict['g'][0]
            m.cx.append(x)
            m.cy.append(y)
            m.area.append(area)
            m.prob.append(0)
            m.num += 1
            m.color.append(3)
        if m.num > 0:
            m.appear = True
        else:
            m.appear = False
        return m
    else:
        m.num = 0
        if len(result_dict[reqColor]) > 0:
            color, r, area, x, y = result_dict[reqColor][0]
            m.cx.append(x)
            m.cy.append(y)
            m.area.append(area)
            m.prob.append(0)
            m.num += 1
            m.color.append(0)
        if m.num > 0:
            m.appear = True
        else:
            m.cx = [0]
            m.cy = [0]
            m.area = [0]
            m.prob = [0]
            m.num = 0
            m.color.append(0)
            m.appear = False
        return m
    publish_result(img_gray, 'gray', '/gray')
    key = cv2.waitKey(1) & 0xff

    # if key == ord('q'):
    #     break
    # rate.sleep()


if __name__ == '__main__':
    rospy.init_node('buoy', anonymous=True)
    topic = "/top/center/image_rect_color/compressed"
    rospy.Subscriber(topic, CompressedImage, callback)
    rospy.Service('vision_bouy', vision_srv_bouy, mission_callback)
    #rospy.spin()
    while not rospy.is_shutdown():
        find_bouy()
