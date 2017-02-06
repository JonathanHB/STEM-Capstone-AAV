#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
import sys
import threading
import time
import math

class Hoop_finder:

	def __init__(self):

		self.sub_image = rospy.Subscriber("/ardrone/front/image_raw", Image, self.takeimage, queue_size=1) #gets front camera raw image
		self.sub_image = rospy.Subscriber("/ardrone/odometry", Odometry, self.odometry, queue_size=1) #gets odometry data
		self.pub_image = rospy.Publisher("~detection_image", Image, queue_size=1) #publishes the image the drone sees, with contours and ellipse superimposed
		self.pub_image2 = rospy.Publisher("~detection_image2", Image, queue_size=1) #used to debug image processing
		self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 1) #publishes commands to drone
		self.bridge = CvBridge()

		self.lastframe = np.zeros((360, 640), dtype = "uint8")
		self.lastflow = np.zeros((360, 640, 2), dtype = "uint8")		
		self.n = 0

	def takeimage(self, img): #runs image processing to find hula hoop, and feeds the resulting drone and hoop pose data into the navigation algorithm
		if (self.n*1.0).is_integer(): 
			self.getvectors(self.processimage1(img)) #posedata = 
		#self.hoopnav(posedata)	
		self.n+=1
	
	
	def odometry(self, data): #unimplemented; the odometry is itself optical flow based
		#stores odometry data to integrate with vision processing
		#print data
		#print data.twist.twist.linear.y
		#self.vy = data.twist.twist.linear.y
		a = 1 #boilerplate code for spacing
				
	def getvectors(self, vectors):

		totalx = 0 #turns into average x/y
		totaly = 0

		totalrad = 0

		scale = 10

		for x in range(0,360,scale):
			for y in range(0,640,scale):

				totalx += vectors[x][y][0]
				totaly += vectors[x][y][1]
				totalrad += vectors[x][y][0]*(x-180) + vectors[x][y][1]*(y-320)

		print totalx/(230400/(scale*scale))
		print totaly/(230400/(scale*scale))
		print totalrad/(230400/(scale*scale))
		

	def processimage1(self, imgdrone1):

		imgbgr = self.bridge.imgmsg_to_cv2(imgdrone1, "bgr8")

		hsv = np.zeros((360, 640, 3), dtype = "uint8")
		hsv[...,1] = 255

    		currentframe = cv2.cvtColor(imgbgr,cv2.COLOR_BGR2GRAY)

    		flow = cv2.calcOpticalFlowFarneback(self.lastframe, currentframe, 0.5, 3, 30, 3, 5, 1.2, 0) #, self.lastflow
		
		"""
		
prev		first 8-bit single-channel input image.
next		second input image of the same size and the same type as prev.
pyr_scale	parameter, specifying the image scale (<1) to build pyramids for each image; pyr_scale=0.5 means a classical pyramid, where each next layer is twice 			smaller than the previous one.
levels		number of pyramid layers including the initial image; levels=1 means that no extra layers are created and only the original images are used.
winsize		averaging window size; larger values increase the algorithm robustness to image noise and give more chances for fast motion detection, but yield more 			blurred motion field.
iterations	number of iterations the algorithm does at each pyramid level.
poly_n		size of the pixel neighborhood used to find polynomial expansion in each pixel; larger values mean that the image will be approximated with smoother 			surfaces, yielding more robust algorithm and more blurred motion field, typically poly_n =5 or 7.
poly_sigma	standard deviation of the Gaussian that is used to smooth derivatives used as a basis for the polynomial expansion; for poly_n=5, you can set 			poly_sigma=1.1, for poly_n=7, a good value would be poly_sigma=1.5.

		"""

		mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])

		#hsv[...,0] = ang*180/np.pi/2
		#hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX) #comment this out once image processing works to save computing power

		hsv[...,0] = 100 + 50*np.sign(flow[...,0])
		hsv[...,2] = 100#cv2.normalize(flow[...,0],None,0,255,cv2.NORM_MINMAX)

    		bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

		self.lastframe = currentframe
		#self.lastflow = flow



		"""
		yellow = [np.array(x, np.uint8) for x in [[25,100,100], [35, 255, 255]]] #the hsv filter used to detect yellow color by cv2.inRange
		#ranges: 0-180,0-255,0-255
		erosion = (5,5) #values used for erosion by cv2.erode
		
		imgrgb = self.bridge.imgmsg_to_cv2(imgdrone1, "bgr8") #converts drone's raw_image to bgr8 used by opencv
		imghsv = cv2.cvtColor(imgrgb, cv2.COLOR_BGR2HSV) #converts hsv to rgb color

		imgyellow = cv2.inRange(imghsv, yellow[0], yellow[1]) #filter the image for yellow, returns 8UC1 binary image
		
		erosion = (5,5)
		imgyellow = cv2.erode(imgyellow, erosion, iterations = 3)
		dilation = (5,5)
		imgyellow = cv2.dilate(imgyellow, dilation, iterations = 5) #erodes, then dilates the image to remove noise points
		"""
			
		imgdrone = self.bridge.cv2_to_imgmsg(imgbgr, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image.publish(imgdrone)

		imgdrone2 = self.bridge.cv2_to_imgmsg(bgr, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image2.publish(imgdrone2)
		
		return flow

if __name__=="__main__":
	rospy.init_node('Hoop_finder')
        
	node = Hoop_finder()

	rospy.spin()
