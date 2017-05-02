#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from scipy import stats
from sensor_msgs.msg import CompressedImage
import math
import dynamic_reconfigure.client
import time
import statistics
from matplotlib import pyplot as plt
img = None
hsv = None
node = None
client = None


def callback(msg):
    global img, hsv
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(arr, 1)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def set_param(param, value):
    global client
    params = {str(param): value}
    config = client.update_configuration(params)


def get_param(param):
    global node
    return rospy.get_param(str(node) + str(param), False)


def get_cv():
    global hsv
    h, s, v = cv2.split(hsv)
    mean = cv2.mean(v)
    sd = cv2.meanStdDev(v, mean)
    return sd / mean


def inrange_ratio(min, ratio, max):
    if min <= ratio <= max:
        return True
    return False


def adjust_exposure_time():
    global hsv, img

    while not rospy.is_shutdown():
        while img is None:
            rospy.sleep(0.01)
            key = cv2.waitKey(1) & 0xff
            if key == ord('q'):
                break

        key = cv2.waitKey(1) & 0xff

        h, s, v = cv2.split(hsv)

        v_1d = v.ravel()

        # MEAN = statistics.mean(v_1d)
        MEAN = cv2.mean(v_1d)[0]
        m = [90, 160]
        _,SD = cv2.meanStdDev(v_1d,MEAN)
        SD = SD[0]
        CV = SD / MEAN
        ev = get_param('exposure')
        try:
            MODE = statistics.mode(v_1d)
        except ValueError:
            MODE = 127
        if MODE >= 230:
            ev -= 1.5
            set_param('exposure', ev)
        elif MODE <= 25:
            ev +=1.5
            set_param('exposure', ev)
        else:
            # # 10% trimmed mean
            # for i in range (0,25):
            #     v_1d.remove(i)
            # for i in range (230,256):
            #     v_1d.remove(i)
            # MEAN = cv2.mean(v_1d)[0]

        print("SD: {0} mode: {1} mean: {2} mean: {3}".format(SD, MODE,MEAN,MEAN1))
        print("CV: {0}".format(CV))
        print("My Auto Exposure: {0}".format(ev))

        cv2.imshow('v', v)
        if key == ord('q'):
            plt.hist(v.ravel(), 256, [0, 256])
            plt.show()
            break

        rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('adjust_exposure_time')
    topic = 'camera/image_raw/compressed'
    # topic = rospy.get_param('', topic)
    node = 'ueye_cam_nodelet/'

    rospy.Subscriber(topic, CompressedImage, callback)
    client = dynamic_reconfigure.client.Client(node)
    set_param('auto_exposure', False)
    set_param('auto_frame_rate', True)
    ev = 33
    print("Exposure Start: {0}".format(ev))
    set_param('exposure', ev)
    time.sleep(2)
    adjust_exposure_time()
