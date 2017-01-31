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

		self.sub_image = rospy.Subscriber("/ardrone/image_raw", Image, self.takeimage, queue_size=1) #gets front camera raw image
		self.sub_image = rospy.Subscriber("/ardrone/odometry", Odometry, self.odometry, queue_size=1) #gets odometry data
		self.pub_image = rospy.Publisher("~detection_image", Image, queue_size=1) #publishes the image the drone sees, with contours and ellipse superimposed
		self.pub_image2 = rospy.Publisher("~detection_image2", Image, queue_size=1) #used to debug image processing
		self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 1) #publishes commands to drone
		self.bridge = CvBridge()

		self.lastframe = np.zeros((360, 640), dtype = "uint8") #changing to float didn't help		

	def takeimage(self, img): #runs image processing to find hula hoop, and feeds the resulting drone and hoop pose data into the navigation algorithm
		
		posedata = self.processimage1(img)
		self.hoopnav(posedata)	

	
	
	def odometry(self, data): #unimplemented; the odometry is itself optical flow based
		#stores odometry data to integrate with vision processing
		#print data
		#print data.twist.twist.linear.y
		#self.vy = data.twist.twist.linear.y
		a = 1 #boilerplate code for spacing

	def processimage1(self, imgdrone1):
		#this method finds the distance and angle of the hoop and the angle of the drone relative to the hoop
		#print "image recieved"

		imgbgr = self.bridge.imgmsg_to_cv2(imgdrone1, "bgr8")

		#ret, frame1 = imgdrone1.read()
		#self.lastframe = cv2.cvtColor(imgdrone1,cv2.COLOR_BGR2GRAY)
		hsv = np.zeros_like(self.lastframe)
		hsv[...,1] = 255
		#while(1):
    			#ret, frame2 = imgdrone1.read()
    		currentframe = cv2.cvtColor(imgbgr,cv2.COLOR_BGR2GRAY)

    		flow = cv2.calcOpticalFlowFarneback(self.lastframe, currentframe, 0.5, 3, 15, 3, 5, 1.2, 0) # 3: None,
    		mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
		print len(mag[2])
    		hsv[...,0] = ang*180/np.pi/2
    		hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
    		bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

		#self.pub_image.publish(bgr)
		self.lastframe = currentframe

    		#cv2.imshow('frame2',bgr)
    		#k = cv2.waitKey(30) & 0xff
    		"""	if k == 27:
    		    	break
    		elif k == ord('s'):
        		cv2.imwrite('opticalfb.png',frame2)
        	cv2.imwrite('opticalhsv.png',bgr)"""



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
			
		imgdrone2 = self.bridge.cv2_to_imgmsg(imgbgr, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image.publish(imgdrone2)
		

if __name__=="__main__":
	rospy.init_node('Hoop_finder')
        
	node = Hoop_finder()

	rospy.spin()
