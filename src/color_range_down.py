#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
import rospkg
from vision_lib import *

ix,iy = -1,-1
t=0
img = None
wait = False

def draw_circle(event,x,y,flags,param):
	global ix,iy,t
	if event == cv2.EVENT_LBUTTONDOWN:
		t=1
		ix,iy = x,y

def create_trackbar():
	width = 1366
	height = 768
	cv2.namedWindow('image',flags=cv2.WINDOW_NORMAL)
	cv2.namedWindow('mask',flags=cv2.WINDOW_NORMAL)
	cv2.namedWindow('orange1',flags=cv2.WINDOW_NORMAL)
	cv2.namedWindow('yellow',flags=cv2.WINDOW_NORMAL)
	#cv2.namedWindow('orange2',flags=cv2.WINDOW_NORMAL)

	cv2.moveWindow('image',0,(height*2/3)+100)
	cv2.moveWindow('mask',width/2,0)
	cv2.moveWindow('orange1',width/2,(height/3)+50)
	cv2.moveWindow('yellow',width/2,(height*2/3)+50)
	#cv2.moveWindow('orange2',width/2,(height*2/3)+50)

	cv2.resizeWindow('image',(width/2)-100,height)

	cv2.createTrackbar('Hmin','image',0,180,nothing)
	cv2.createTrackbar('Smin','image',0,255,nothing)
	cv2.createTrackbar('Vmin','image',0,255,nothing)
	cv2.createTrackbar('Hmax','image',0,180,nothing)
	cv2.createTrackbar('Smax','image',0,255,nothing)
	cv2.createTrackbar('Vmax','image',0,255,nothing)
	cv2.setTrackbarPos('Hmin','image',0)
	cv2.setTrackbarPos('Smin','image',0)
	cv2.setTrackbarPos('Vmin','image',0)
	cv2.setTrackbarPos('Hmax','image',180)
	cv2.setTrackbarPos('Smax','image',255)
	cv2.setTrackbarPos('Vmax','image',255)
	
	cv2.setMouseCallback('image',draw_circle)

def select_color():
	global ix,iy,t,img,wait

	x = Finding_Color()

	main_color = x.color

	create_trackbar()

	h,s,v = 0,0,0
	Hmax = [180]
	Smax = [255]
	Vmax = [255]
	Hmin = [0]
	Smin = [0]
	Vmin = [0]
	Hmax_re = []
	Smax_re = []
	Vmax_re = []
	Hmin_re = []
	Smin_re = []
	Vmin_re = []
	while(img is None):
		rospy.sleep(0.01)
	
	while not rospy.is_shutdown():
		
		hsv = process_img_down(img)
		bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
		cv2.imshow('img_bgr',bgr)
		
		k = cv2.waitKey(1) & 0xff
		if k == ord('p') and wait == False  and t != 1:
			wait = True
		elif k == ord('p') and wait == True  and t != 1:
			wait = False

		if(t==1):
			Hmax_re=[]
			Smax_re=[]
			Vmax_re=[]
			Hmin_re=[]
			Smin_re=[]
			Vmin_re=[]
			i,j,k= hsv[iy,ix]

			Hmax.append(max(i,Hmax[-1]))
			Smax.append(max(j,Smax[-1]))
			Vmax.append(max(k,Vmax[-1]))
			Hmin.append(min(i,Hmin[-1]))
			Smin.append(min(j,Smin[-1]))
			Vmin.append(min(k,Vmin[-1]))

			cv2.setTrackbarPos('Hmin', 'image', Hmin[-1])
			cv2.setTrackbarPos('Smin', 'image', Smin[-1])
			cv2.setTrackbarPos('Vmin', 'image', Vmin[-1])
			cv2.setTrackbarPos('Hmax', 'image', Hmax[-1])
			cv2.setTrackbarPos('Smax', 'image', Smax[-1])
			cv2.setTrackbarPos('Vmax', 'image', Vmax[-1])

		if(Hmin[-1] != cv2.getTrackbarPos('Hmin','image') or Smin[-1] != cv2.getTrackbarPos('Smin','image') or Vmin[-1] != cv2.getTrackbarPos('Vmin','image') or Hmax[-1] != cv2.getTrackbarPos('Hmax','image') or Smax[-1] != cv2.getTrackbarPos('Smax','image') or Vmax[-1] != cv2.getTrackbarPos('Vmax','image')):

			Hmin.append(cv2.getTrackbarPos('Hmin','image'))
			Smin.append(cv2.getTrackbarPos('Smin','image'))
			Vmin.append(cv2.getTrackbarPos('Vmin','image'))
			Hmax.append(cv2.getTrackbarPos('Hmax','image'))
			Smax.append(cv2.getTrackbarPos('Smax','image'))
			Vmax.append(cv2.getTrackbarPos('Vmax','image'))


		main_color.lower_main = [Hmin[-1],Smin[-1],Vmin[-1]]
		main_color.upper_main = [Hmax[-1],Smax[-1],Vmax[-1]]

		lower = np.array(main_color.lower_main,np.uint8)
		upper = np.array(main_color.upper_main,np.uint8) 

		lower_orange1 = np.array(main_color.lower_orange1,np.uint8)
		upper_orange1 = np.array(main_color.upper_orange1,np.uint8)

		lower_orange2 = np.array(main_color.lower_orange2,np.uint8)
		upper_orange2 = np.array(main_color.upper_orange2,np.uint8)

		lower_yellow = np.array(main_color.lower_yellow,np.uint8)
		upper_yellow = np.array(main_color.upper_yellow,np.uint8)

		mask = cv2.inRange(hsv,lower, upper)
		mask_orange2 = cv2.inRange(hsv,lower_orange2, upper_orange2)
		mask_orange1 = cv2.inRange(hsv,lower_orange1, upper_orange1)
		mask_yellow = cv2.inRange(hsv,lower_yellow, upper_yellow)

		if k == 27:
			break
		# undo
		elif( k == ord('z') or k == ord('Z') )and len(Hmax)-1 >0  and t != 1:
			Hmax_re.append(Hmax.pop(-1))
			Smax_re.append(Smax.pop(-1))
			Vmax_re.append(Vmax.pop(-1))
			Hmin_re.append(Hmin.pop(-1))
			Smin_re.append(Smin.pop(-1))
			Vmin_re.append(Vmin.pop(-1))

			cv2.setTrackbarPos('Hmin', 'image', Hmin[-1])
			cv2.setTrackbarPos('Smin', 'image', Smin[-1])
			cv2.setTrackbarPos('Vmin', 'image', Vmin[-1])
			cv2.setTrackbarPos('Hmax', 'image', Hmax[-1])
			cv2.setTrackbarPos('Smax', 'image', Smax[-1])
			cv2.setTrackbarPos('Vmax', 'image', Vmax[-1])
		# redo
		elif( k == ord('x') or k == ord('X')) and len(Hmax_re)-1>0  and t != 1:
			Hmax.append(Hmax_re.pop(-1))
			Smax.append(Smax_re.pop(-1))
			Vmax.append(Vmax_re.pop(-1))
			Hmin.append(Hmin_re.pop(-1))
			Smin.append(Smin_re.pop(-1))
			Vmin.append(Vmin_re.pop(-1))

			cv2.setTrackbarPos('Hmin', 'image', Hmin[-1])
			cv2.setTrackbarPos('Smin', 'image', Smin[-1])
			cv2.setTrackbarPos('Vmin', 'image', Vmin[-1])
			cv2.setTrackbarPos('Hmax', 'image', Hmax[-1])
			cv2.setTrackbarPos('Smax', 'image', Smax[-1])
			cv2.setTrackbarPos('Vmax', 'image', Vmax[-1])
		
		elif k == ord("1") and t != 1:
			main_color = x.color

			Hmax = [0]
			Smax = [0]
			Vmax = [0]
			Hmin = [180]
			Smin = [255]
			Vmin = [255]

			Hmax_re = []
			Smax_re = []
			Vmax_re = []
			Hmin_re = []
			Smin_re = []
			Vmin_re = []

			cv2.setTrackbarPos('Hmin', 'image', Hmin[-1])
			cv2.setTrackbarPos('Smin', 'image', Smin[-1])
			cv2.setTrackbarPos('Vmin', 'image', Vmin[-1])
			cv2.setTrackbarPos('Hmax', 'image', Hmax[-1])
			cv2.setTrackbarPos('Smax', 'image', Smax[-1])
			cv2.setTrackbarPos('Vmax', 'image', Vmax[-1])

		elif k == ord("y") and t != 1:
			if main_color.check_yellow:
				main_color.lower_yellow = [ Hmin[-1] , Smin[-1] , Vmin[-1] ]
				main_color.upper_yellow = [ Hmax[-1] , Smax[-1] , Vmax[-1] ]

				main_color.check_orange2 = False
				main_color.check_orange1 = False
				main_color.check_yellow = False

			elif not main_color.check_yellow:
				main_color.lower_main = main_color.lower_yellow[:]
				main_color.upper_main = main_color.upper_yellow[:]


				Hmin.append(main_color.lower_yellow[0])
				Smin.append(main_color.lower_yellow[1])
				Vmin.append(main_color.lower_yellow[2])

				Hmax.append(main_color.upper_yellow[0])
				Smax.append(main_color.upper_yellow[1])
				Vmax.append(main_color.upper_yellow[2])

				cv2.setTrackbarPos('Hmin', 'image', Hmin[-1])
				cv2.setTrackbarPos('Smin', 'image', Smin[-1])
				cv2.setTrackbarPos('Vmin', 'image', Vmin[-1])
				cv2.setTrackbarPos('Hmax', 'image', Hmax[-1])
				cv2.setTrackbarPos('Smax', 'image', Smax[-1])
				cv2.setTrackbarPos('Vmax', 'image', Vmax[-1])

				main_color.check_orange1 = False
				main_color.check_orange2 = False
				main_color.check_yellow = True

		elif k == ord("o") and t != 1:
			if main_color.check_orange1:
				main_color.lower_orange1 = [ Hmin[-1] , Smin[-1] , Vmin[-1] ]
				main_color.upper_orange1 = [ Hmax[-1] , Smax[-1] , Vmax[-1] ]

				main_color.check_orange1 = False
				main_color.check_orange2 = False
				main_color.check_yellow = False

			elif not main_color.check_orange1:
				main_color.lower_main = main_color.lower_orange1[:]
				main_color.upper_main = main_color.upper_orange1[:]


				Hmin.append(main_color.lower_orange1[0])
				Smin.append(main_color.lower_orange1[1])
				Vmin.append(main_color.lower_orange1[2])

				Hmax.append(main_color.upper_orange1[0])
				Smax.append(main_color.upper_orange1[1])
				Vmax.append(main_color.upper_orange1[2])

				cv2.setTrackbarPos('Hmin', 'image', Hmin[-1])
				cv2.setTrackbarPos('Smin', 'image', Smin[-1])
				cv2.setTrackbarPos('Vmin', 'image', Vmin[-1])
				cv2.setTrackbarPos('Hmax', 'image', Hmax[-1])
				cv2.setTrackbarPos('Smax', 'image', Smax[-1])
				cv2.setTrackbarPos('Vmax', 'image', Vmax[-1])

				main_color.check_orange1 = True
				main_color.check_orange2 = False
				main_color.check_yellow = False

		

		elif k == ord("s") and t != 1:
			x.save()
		t=0
		cv2.imshow('image',hsv)
		cv2.imshow('mask',mask)
		#cv2.imshow('orange2',mask_orange2)
		cv2.imshow('orange1',mask_orange1)
		cv2.imshow('yellow',mask_yellow)

	cv2.destroyAllWindows()


def callback(msg):
	global img,wait
	# print msg
	if wait == False:
		arr = np.fromstring(msg.data,np.uint8)
		img = cv2.imdecode(arr,1)
		img = cv2.resize(img,(320,256))

class Color:
	def __init__(self):
		self.check_orange2 = False
		self.check_orange1 = False
		self.check_yellow = False
		self.name = 'color_down'
		self.upper_orange2 = rospy.get_param("color_range/color_down/upper_orange2",[0,0,0])
		self.lower_orange2 = rospy.get_param("color_range/color_down/lower_orange2",[180,255,255])
		self.upper_orange1 = rospy.get_param("color_range/color_down/upper_orange1",[0,0,0])
		self.lower_orange1 = rospy.get_param("color_range/color_down/lower_orange1",[180,255,255])
		self.upper_yellow = rospy.get_param("color_range/color_down/upper_yellow",[0,0,0])
		self.lower_yellow = rospy.get_param("color_range/color_down/lower_yellow",[180,255,255])
		self.lower_main = [0,0,0]
		self.upper_main = [180,255,255]

class Finding_Color:
	def __init__(self):
		self.color = Color()
		self.path = rospkg.RosPack().get_path('zeabus_vision_sk')

	def save(self):
		f = open(self.path+"/params/color_down.yaml", "w")
		x = self.genyaml(self.color)
		f.write(x)
		f.close()
		try :
			for i in range (0,3):
				rospy.set_param('/color_range/color_down/lower_orange1',self.color.lower_orange1)
				rospy.set_param('/color_range/color_down/upper_orange1',self.color.upper_orange1)
		
				rospy.set_param('/color_range/color_down/lower_orange2',self.color.lower_orange2)
				rospy.set_param('/color_range/color_down/upper_orange2',self.color.upper_orange2)
				
				rospy.set_param('/color_range/color_down/lower_yellow',self.color.lower_yellow)
				rospy.set_param('/color_range/color_down/upper_yellow',self.color.upper_yellow)
				
			print 'save'	
		except Exception:
			print 'not save' 

	def genyaml(self,obj):
		# print obj.name
		tmp = ''.join([obj.name + ":\n" ,
				" " + "upper_yellow: " + str(list(obj.upper_yellow)) + "\n\n" ,
				" " + "lower_yellow: " + str(list(obj.lower_yellow)) + "\n\n" ,
				" " + "upper_orange1: " + str(list(obj.upper_orange1)) + "\n\n" ,
				" " + "lower_orange1: " + str(list(obj.lower_orange1)) + "\n\n" ,
				" " + "upper_orange2: " + str(list(obj.upper_orange2)) + "\n\n" ,
				" " + "lower_orange2: " + str(list(obj.lower_orange2)) ])
		return tmp


if __name__ == '__main__':
	rospy.init_node('color_range_down')
	topic = rospy.get_param('color_range/topic_down','/rightcam_bottom/image_raw/compressed')
	# topic = rospy.get_param('color_range/topic_down','/leftcam_bottom/image_raw/compressed')
	rospy.Subscriber(topic,CompressedImage,callback)
	select_color()
