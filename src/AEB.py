#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage
import dynamic_reconfigure.client

img = None
hsv = None

def callback(msg):
    global img, hsv
    if wait == False:
        arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(arr, 1)
        img = cv2.resize(img, (320, 256))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

def set_exposure(value):

def main():


if __name__ == '__main__':
    rospy.init_node('AEB')
    topic = "/camera/image_raw/compressed"
    rospy.Subscriber(topic, CompressedImage, callback)
    main()
