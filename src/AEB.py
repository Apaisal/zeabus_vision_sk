#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage
import dynamic_reconfigure.client
import time
import math

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


def get_param(param):
    return rospy.get_param('ueye_cam_nodelet/' + str(param), False)


def main():
    global client, img
    images = []
    delta = 25
    set_param('auto_exposure', True)
    set_param('auto_frame_rate', True)
    ev_auto = get_param('exposure')
    print("EV_auto: {0}".format(ev_auto))
    set_param('auto_exposure', False)
    # exposure = [ev_auto - delta, ev_auto, ev_auto + delta]
    exposure = [ev_auto - delta, ev_auto + delta]
    for ev in exposure:
        t = time.time()
        set_param('exposure', int(ev))
        delta_t = time.time() - t
        print("time: {0}".format(delta_t))
        time.sleep(1)
        name = 'image exposure :' + str(ev)
        images.append(img.copy())
        # EV = log2(f^2 / t)
        # et = math.pow(f, 2.0) / math.pow(2.0, ev)
        cv2.imshow(name, img.copy())

    exposure_times = np.array(exposure, dtype=np.float32)
# debvec
    merge_debvec = cv2.createMergeDebevec()
    hdr_debvec = merge_debvec.process(images, times=exposure_times.copy())
# robertson
    merge_robertson = cv2.createMergeRobertson()
    hdr_robertson = merge_robertson.process(
        images, times=exposure_times.copy())

    tonemap1 = cv2.createTonemapDurand(gamma=2.2)
    res_debvec = tonemap1.process(hdr_debvec.copy())
    tonemap2 = cv2.createTonemapDurand(gamma=1.3)
    res_robertson = tonemap2.process(hdr_robertson.copy())

#  mertens not
    merge_mertens = cv2.createMergeMertens()
    res_mertens = merge_mertens.process(images)

    cv2.imshow('debvec', res_debvec)
    cv2.imshow('robertson', res_robertson)
    cv2.imshow('mertens', res_mertens)

    while True:
        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            break

if __name__ == '__main__':
    rospy.init_node('AEB', anonymous=True)
    topic = "/camera/image_raw/compressed"
    rospy.Subscriber(topic, CompressedImage, callback)
    main()
