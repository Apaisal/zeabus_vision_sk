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
	cv2.namedWindow('red',flags=cv2.WINDOW_NORMAL)
	cv2.namedWindow('white',flags=cv2.WINDOW_NORMAL)
	cv2.namedWindow('orange',flags=cv2.WINDOW_NORMAL)
	cv2.namedWindow('yellow',flags=cv2.WINDOW_NORMAL)
	
	cv2.moveWindow('image',0,0)
	cv2.moveWindow('mask',width/2,0)
	cv2.moveWindow('red',width/2,(height*2/3)+60)
	cv2.moveWindow('white',width,0)
	cv2.moveWindow('orange',width,(height*2/3)+60)
	cv2.moveWindow('yellow',width*2,(height*2/3)+60)
	
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
		
		lower_orange = np.array(main_color.lower_orange,np.uint8)
		upper_orange = np.array(main_color.upper_orange,np.uint8)

		lower_yellow = np.array(main_color.lower_yellow,np.uint8)
		upper_yellow = np.array(main_color.upper_yellow,np.uint8)

		lower_red = np.array(main_color.lower_red,np.uint8)
		upper_red = np.array(main_color.upper_red,np.uint8)

		lower_white = np.array(main_color.lower_white,np.uint8)
		upper_white = np.array(main_color.upper_white,np.uint8)

		mask = cv2.inRange(hsv,lower, upper)
		mask_orange = cv2.inRange(hsv,lower_orange, upper_orange)
		mask_yellow = cv2.inRange(hsv,lower_yellow, upper_yellow)
		mask_red = cv2.inRange(hsv,lower_red, upper_red)
		mask_white = cv2.inRange(hsv,lower_white, upper_white)

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

				main_color.check_orange = False
				main_color.check_yellow = False
				main_color.check_white = False
				main_color.check_red = False

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

				main_color.check_orange = False
				main_color.check_yellow = True
				main_color.check_white = False
				main_color.check_red = False

		elif k == ord("o") and t != 1:
			if main_color.check_orange:
				main_color.lower_orange = [ Hmin[-1] , Smin[-1] , Vmin[-1] ]
				main_color.upper_orange = [ Hmax[-1] , Smax[-1] , Vmax[-1] ]

				main_color.check_orange = False
				main_color.check_yellow = False
				main_color.check_white = False
				main_color.check_red = False

			elif not main_color.check_orange:
				main_color.lower_main = main_color.lower_orange[:]
				main_color.upper_main = main_color.upper_orange[:]


				Hmin.append(main_color.lower_orange[0])
				Smin.append(main_color.lower_orange[1])
				Vmin.append(main_color.lower_orange[2])

				Hmax.append(main_color.upper_orange[0])
				Smax.append(main_color.upper_orange[1])
				Vmax.append(main_color.upper_orange[2])

				cv2.setTrackbarPos('Hmin', 'image', Hmin[-1])
				cv2.setTrackbarPos('Smin', 'image', Smin[-1])
				cv2.setTrackbarPos('Vmin', 'image', Vmin[-1])
				cv2.setTrackbarPos('Hmax', 'image', Hmax[-1])
				cv2.setTrackbarPos('Smax', 'image', Smax[-1])
				cv2.setTrackbarPos('Vmax', 'image', Vmax[-1])

				main_color.check_orange = True
				main_color.check_yellow = False
				main_color.check_white = False
				main_color.check_red = False
		
		elif k == ord("r") and t != 1:
			if main_color.check_red:
				main_color.lower_red = [ Hmin[-1] , Smin[-1] , Vmin[-1] ]
				main_color.upper_red = [ Hmax[-1] , Smax[-1] , Vmax[-1] ]

				main_color.check_orange = False
				main_color.check_yellow = False
				main_color.check_white = False
				main_color.check_red = False

			elif not main_color.check_red:
				main_color.lower_main = main_color.lower_red[:]
				main_color.upper_main = main_color.upper_red[:]


				Hmin.append(main_color.lower_red[0])
				Smin.append(main_color.lower_red[1])
				Vmin.append(main_color.lower_red[2])

				Hmax.append(main_color.upper_red[0])
				Smax.append(main_color.upper_red[1])
				Vmax.append(main_color.upper_red[2])

				cv2.setTrackbarPos('Hmin', 'image', Hmin[-1])
				cv2.setTrackbarPos('Smin', 'image', Smin[-1])
				cv2.setTrackbarPos('Vmin', 'image', Vmin[-1])
				cv2.setTrackbarPos('Hmax', 'image', Hmax[-1])
				cv2.setTrackbarPos('Smax', 'image', Smax[-1])
				cv2.setTrackbarPos('Vmax', 'image', Vmax[-1])

				main_color.check_orange = False
				main_color.check_yellow = False
				main_color.check_white = False
				main_color.check_red = True
		
		elif k == ord("w") and t != 1:
			if main_color.check_white:
				main_color.lower_white = [ Hmin[-1] , Smin[-1] , Vmin[-1] ]
				main_color.upper_white = [ Hmax[-1] , Smax[-1] , Vmax[-1] ]

				main_color.check_orange = False
				main_color.check_yellow = False
				main_color.check_white = False
				main_color.check_red = False

			elif not main_color.check_white:
				main_color.lower_main = main_color.lower_white[:]
				main_color.upper_main = main_color.upper_white[:]


				Hmin.append(main_color.lower_white[0])
				Smin.append(main_color.lower_white[1])
				Vmin.append(main_color.lower_white[2])

				Hmax.append(main_color.upper_white[0])
				Smax.append(main_color.upper_white[1])
				Vmax.append(main_color.upper_white[2])

				cv2.setTrackbarPos('Hmin', 'image', Hmin[-1])
				cv2.setTrackbarPos('Smin', 'image', Smin[-1])
				cv2.setTrackbarPos('Vmin', 'image', Vmin[-1])
				cv2.setTrackbarPos('Hmax', 'image', Hmax[-1])
				cv2.setTrackbarPos('Smax', 'image', Smax[-1])
				cv2.setTrackbarPos('Vmax', 'image', Vmax[-1])

				main_color.check_orange = False
				main_color.check_yellow = False
				main_color.check_white = True
				main_color.check_red = False
		

		elif k == ord("s") and t != 1:
			x.save()
		t=0
		cv2.imshow('image',hsv)
		cv2.imshow('mask',mask)
		cv2.imshow('orange',mask_orange)
		cv2.imshow('yellow',mask_yellow)
		cv2.imshow('red',mask_red)
		cv2.imshow('white',mask_white)

	cv2.destroyAllWindows()


def callback(msg):
	global img,wait

	if wait == False:
		arr = np.fromstring(msg.data,np.uint8)
		img = cv2.imdecode(arr,1)
		img = cv2.resize(img,(320,256))

class Color:
	def __init__(self):
		self.check_orange = False
		self.check_yellow = False
		self.check_red = False
		self.check_white = False
		self.name = 'color_down'
		
		self.upper_orange = rospy.get_param("color_range/color_down/upper_orange",[0,0,0])
		self.lower_orange = rospy.get_param("color_range/color_down/lower_orange",[180,255,255])
		
		self.upper_yellow = rospy.get_param("color_range/color_down/upper_yellow",[0,0,0])
		self.lower_yellow = rospy.get_param("color_range/color_down/lower_yellow",[180,255,255])
		
		self.upper_red = rospy.get_param("color_range/color_down/upper_red",[0,0,0])
		self.lower_red = rospy.get_param("color_range/color_down/lower_red",[180,255,255])
		
		self.upper_white = rospy.get_param("color_range/color_down/upper_white",[0,0,0])
		self.lower_white = rospy.get_param("color_range/color_down/lower_white",[180,255,255])
		
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
				rospy.set_param('/color_range/color_down/lower_orange',self.color.lower_orange)
				rospy.set_param('/color_range/color_down/upper_orange',self.color.upper_orange)
		
				rospy.set_param('/color_range/color_down/lower_yellow',self.color.lower_yellow)
				rospy.set_param('/color_range/color_down/upper_yellow',self.color.upper_yellow)

				rospy.set_param('/color_range/color_down/lower_red',self.color.lower_red)
				rospy.set_param('/color_range/color_down/upper_red',self.color.upper_red)

				rospy.set_param('/color_range/color_down/lower_white',self.color.lower_white)
				rospy.set_param('/color_range/color_down/upper_white',self.color.upper_white)
				
			print 'save'	
		except Exception:
			print 'not save' 

	def genyaml(self,obj):
		# print obj.name
		tmp = ''.join([obj.name + ":\n" ,
				" " + "upper_yellow: " + str(list(obj.upper_yellow)) + "\n\n" ,
				" " + "lower_yellow: " + str(list(obj.lower_yellow)) + "\n\n" ,
				" " + "upper_orange: " + str(list(obj.upper_orange)) + "\n\n" ,
				" " + "lower_orange: " + str(list(obj.lower_orange)) + "\n\n" ,
				" " + "upper_red: " + str(list(obj.upper_red)) + "\n\n" ,
				" " + "lower_red: " + str(list(obj.lower_red)) + "\n\n" ,
				" " + "upper_white: " + str(list(obj.upper_white)) + "\n\n" ,
				" " + "lower_white: " + str(list(obj.lower_white)) ])
		return tmp

if __name__ == '__main__':
	rospy.init_node('color_range_down')
	topic = rospy.get_param('color_range/topic_down','/rightcam_bottom/image_raw/compressed')
	# topic = rospy.get_param('color_range/topic_down','/leftcam_bottom/image_raw/compressed')
	rospy.Subscriber(topic,CompressedImage,callback)
	select_color()