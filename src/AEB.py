#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage
import dynamic_reconfigure.client
import time

img = None
hsv = None
client = None
wait = False


def callback(msg):
    global img, hsv
    if wait == False:
        arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(arr, 1)
        img = cv2.resize(img, (320, 256))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def set_param(param, value):
    global client
    client = dynamic_reconfigure.client.Client('ueye_cam_nodelet')
    params = {str(param): value}
    config = client.update_configuration(params)


def main():
    global client, img
    exposure = [15, 30, 45]
    images = []
    set_param('auto_frame_rate', True)
    set_param('auto_exposure', False)
    for ev in exposure:
        t = time.time()
        set_param('exposure', int(ev))
        delta_t = time.time() - t
        print("time: {0}".format(delta_t))
        time.sleep(1.5)
        name = 'image exposure :' + str(ev)
        images.append(img.copy())
        cv2.imshow(name, img.copy())
        
    cv2.waitKey(30000)


if __name__ == '__main__':
    rospy.init_node('AEB', anonymous=True)
    topic = "/camera/image_raw/compressed"
    rospy.Subscriber(topic, CompressedImage, callback)
    main()
