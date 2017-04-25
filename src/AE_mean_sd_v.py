#!usr/bin/env python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage

img = None
hsv = None
node = None
client = None


def callback(msg):
    global img, hsv
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecoder(arr, 1)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def set_param(param, value):
    global client
    params = {str(param): value}
    config = client.update_configuration(params)


def get_param(param):
    global node
    return rospy.get_param(str(node) + str(param), False)


def get_ratio():
    global hsv
    h, s, v = cv2.split(hsv)
    mean = cv2.mean(v)
    sd = cv2.meanStdDev(v, mean)
    return mean * (1 / sd)


def inrange_ratio(min, ratio, max):
    if min <= ratio <= max:
        return True
    return False


def adjust_exposure_time():
    global hsv, img
    while img is None:
        rospy.sleep(0.01)
        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            break

    ratio_current = get_ratio()

    while not rospy.is_shutdown():
        key = cv2.waitKey(1) & 0xff

        while img is None:
            rospy.sleep(0.01)
            key = cv2.waitKey(1) & 0xff
            if key == ord('q'):
                break

        if key == ord('q'):
            break

        ratio_new = get_ratio()
        ratio_min = 35
        ratio_max = 70
        if inrange_ratio(ratio_min,ratio_new,ratio_max)
            h, s, v = cv2.split(hsv)
            mean = cv2.mean(v)
            sd = cv2.meanStdDev(v, mean)

            exposure_current = get_param('exposure')
            exposure_new = exposure_current

        cv2.imshow('img', img)
        cv2.imshow('v', v)

if __name__ == '__main__':
    rospy.init_node('adjust_exposure_time')
    topic = 'camera/image_raw/compressed'
    topic = rospy.get_param('', topic)
    node = 'ueye_cam_nodelet/'

    rospy.Subscriber(topic, CompressedImage, callback)
    client = dynamic_reconfigure.client.Client(node)
    set_param('auto_exposure', True)
    rospy.sleep(2)
    set_param('auto_expousre', False)
    adjust_exposure_time()
