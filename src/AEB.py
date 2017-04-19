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


def set_exposure(value):
    global client
    client = dynamic_reconfigure.client.Client('ueye_cam_nodelet')
    if value != -1:
        params = {'auto_exposure': False}
        params = {'exposure': value}
    else:
        params = {'auto_exposure': True}
    config = client.update_configuration(params)


def main():
    global client, img
    exposure = [0, 15, 30, -1]
    images = []
    for ev in exposure:
        t = time.time()
        set_exposure(ev)
        delta_t = time.time() - t
        # time: 0.0346131324768
        # time: 0.0417408943176
        # time: 0.0291020870209
        print("time: {0}".format(delta_t))
        time.sleep(2)
        name = 'image exposure :' + str(ev)
        cv2.imshow(name, img.copy())
    cv2.waitKey(30000)

if __name__ == '__main__':
    rospy.init_node('AEB', anonymous=True)
    topic = "/camera/image_raw/compressed"
    rospy.Subscriber(topic, CompressedImage, callback)
    main()
