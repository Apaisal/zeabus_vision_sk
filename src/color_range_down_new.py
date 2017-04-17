#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage

from vision_lib import *

global pixel, click, img, wait, hsv
pixel = {}
pixel['x'], pixel['y'] = -1, -1
click = False
img = None
hsv = None
wait = False


def callback(msg):
    global img, wait, hsv

    if wait == False:
        arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(arr, 1)
        img = cv2.resize(img, (320, 256))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def draw_circle(event, x, y, flags, param):
    global pixel, click
    if event == cv2.EVENT_LBUTTONDOWN:
        click = True
        pixel['x'], pixel['y'] = x, y


class window:

    def __init__(self):
        self.width = 1366
        self.height = 768
        self.size = 250
        self.x = self.width / 3
        self.y = 0
        self.lower = {}
        self.upper = {}
        self.lower_tmp = {}
        self.upper_tmp = {}
        self.select = {}
        self.path = rospkg.RosPack().get_path('zeabus_vision_sk')

    def create(self, name):
        cv2.namedWindow(name, flags=cv2.WINDOW_NORMAL)
        cv2.moveWindow(name, self.x + self.x / 5, self.y + self.y / 5)
        cv2.resizeWindow(name, self.size, self.size)
        self.update_position()
        self.create_range(name)

    def update_position(self):

        self.y += self.size

        if self.y + self.size >= self.height:
            self.x += self.size
            self.y = 0

    def create_range(self, name):
        self.param_lower = rospy.get_param(
            "color_range/down/lower_" + name, [180, 255, 255])
        self.param_upper = rospy.get_param(
            "color_range/down/upper_" + name, [0, 0, 0])
        self.lower[name] = [[0, 0, 0]]
        self.upper[name] = [[180, 255, 255]]
        self.lower_tmp[name] = []
        self.upper_tmp[name] = []
        self.select[name] = False

    def get_param(self, name):
        self.param_lower = rospy.get_param(
            "color_range/down/lower_" + name, [180, 255, 255])
        self.param_upper = rospy.get_param(
            "color_range/down/upper_" + name, [0, 0, 0])
        return self.param_lower, self.param_upper

    def push_range(self, name, lower, upper):
        self.lower[name].append([lower])
        self.upper[name].append([upper])

    def get_range(self, name):
        return self.lower[name][-1], self.upper[name][-1]

    def undo_range(self, name):
        if len(self.lower[name]) > 0:
            self.lower_tmp[name].append(self.lower[name][-1])
            self.upper_tmp[name].append(self.upper[name][-1])
            self.lower[name].pop()
            self.upper[name].pop()
            print('undo')
        else:
            print('cannot undo')

    def redo_range(self, name):
        if len(self.lower_tmp[name]) > 0:
            self.lower[name].append(self.lower_tmp[name][-1])
            self.upper[name].append(self.upper_tmp[name][-1])
            self.lower_tmp[name].pop()
            self.upper_tmp[name].pop()
            print('redo')
        else:
            print('cannot redo')

    def show_image(self, name):
        global hsv
        result = cv2.inRange(hsv, np.array(self.lower[name][-1], np.uint8),
                             np.array(self.upper[name][-1], np.uint8))
        cv2.imshow(name, result)

    def save(self):

        try:
            for i in range(0, 3):
                for name in self.lower:
                    rospy.set_param(
                        '/color_range/color_down/lower_orange', self.lower[name][-1])
                    rospy.set_param(
                        '/color_range/color_down/upper_orange', self.upper[name][-1])

                f = open(self.path + "/params/color_down.yaml", "w")
                x = self.genyaml()
                f.write(x)
                f.close()
            print 'save'
        except Exception:
            print 'not save'

    def genyaml(self):
        tmp = " color_down:\n"
        for name in self.lower:
            tmp += " " + "upper_" + name + ": " + str(self.upper[name][-1]) + "\n\n" +\
                " " + "lower_" + name + ": " + \
                str(self.lower[name][-1]) + "\n\n"
        print(tmp)
        return tmp


def has_color(window_name, k):
    for name in window_name:
        if k == ord(name[0]):
            return name, True
    return None, False


def set_trackbar(lower, upper):
    [hmin, smin, vmin], [hmax, smax, vmax] = lower, upper
    cv2.setTrackbarPos('Hmin', 'image', hmin)
    cv2.setTrackbarPos('Smin', 'image', smin)
    cv2.setTrackbarPos('Vmin', 'image', vmin)
    cv2.setTrackbarPos('Hmax', 'image', hmax)
    cv2.setTrackbarPos('Smax', 'image', smax)
    cv2.setTrackbarPos('Vmax', 'image', vmax)


def select_color():
    global pixel, t, img, wait, hsv, click

    window_name = ['mask', 'red', 'orange', 'white', 'yellow']

    width = 1366
    height = 768

    w = window()

    cv2.namedWindow('image', flags=cv2.WINDOW_NORMAL)
    cv2.moveWindow('image', 0, 0)
    cv2.resizeWindow('image', (width / 3), height)

    for name in window_name:
        w.create(name)

    cv2.createTrackbar('Hmin', 'image', 0, 180, nothing)
    cv2.createTrackbar('Smin', 'image', 0, 255, nothing)
    cv2.createTrackbar('Vmin', 'image', 0, 255, nothing)
    cv2.createTrackbar('Hmax', 'image', 0, 180, nothing)
    cv2.createTrackbar('Smax', 'image', 0, 255, nothing)
    cv2.createTrackbar('Vmax', 'image', 0, 255, nothing)
    cv2.createTrackbar('m <-> c', 'image', 0, 2, nothing)

    set_trackbar([180, 255, 255], [0, 0, 0])
    cv2.setTrackbarPos('m <-> c', 'image', 1)

    cv2.setMouseCallback('image', draw_circle)
    while(img is None):
        rospy.sleep(0.01)

    while not rospy.is_shutdown():

        key = cv2.waitKey(1) & 0xff
        if key == ord('p') and wait == False and not click:
            wait = True
        elif key == ord('p') and wait == True and not click:
            wait = False

        name, status = has_color(window_name, key)

        if click:
            h, s, v = hsv[pixel['x'], pixel['y']]
            [hl, sl, vl], [hu, su, vu] = w.get_range('mask')
            lower_current = [min(h, hl), min(s, sl), min(s, sl)]
            upper_current = [max(h, hu), max(s, su), max(s, su)]

            w.push_range('mask', lower_current, upper_current)

            set_trackbar(lower_current, upper_current)

        elif status:
            if w.select[name]:
                lower_current = [Hmin[-1], Smin[-1], Vmin[-1]]
                upper_current = [Hmax[-1], Smax[-1], Vmax[-1]]
                w.push_range(name, lower_current, upper_current)
            else:
                lower_current, upper_current = w.get_param(name)
                w.push_range('mask', lower_current, upper_current)
                set_trackbar(lower_current, upper_current)

            w.select[name] = not w.select[name]
        #  <-
        elif key == 85:
            w.redo_range('mask')
        # ->
        elif key == 86:
            w.undo_range('mask')
        elif key == ord('s'):
            w.save()
        elif key == ord('q'):
            break

        for name in window_name:
            w.show_image(name)
        cv2.imshow('image', img)

        click = False

    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('color_range_down')
    topic = rospy.get_param('color_range/topic_down',
                            '/rightcam_bottom/image_raw/compressed')
    rospy.Subscriber(topic, CompressedImage, callback)
    select_color()
