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
		self.pub_image = rospy.Publisher("~detection_image", Image, queue_size=1) #publishes the drone camera image, with feature points and visual debugging images superimposed
		self.pub_image2 = rospy.Publisher("~detection_image2", Image, queue_size=1) #used to debug image processing
		self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 1) #publishes commands to drone
		self.pub_takeoff = rospy.Publisher('/ardrone/takeoff',Empty,queue_size=1) #makes drone take off	

		self.bridge = CvBridge()

		self.imagex = 640
		self.imagey = 360 #dimensions of the image, in pixels

		self.ctrx = int(self.imagex/2.0)
		self.ctry = int(self.imagey/2.0) #coordinates of image center, in pixels

		self.old_gray = np.zeros((self.imagex, self.imagey), dtype = "uint8") #grayscale version of previous image used to compute flow !!
		self.mask = np.zeros((self.imagex, self.imagey, 3), dtype = "uint8") #stores point trails !!


		self.v = [] #horizontal velocity from bottom camera

		self.initframe = True #used to initialize point set once on the first camera frame	

		self.trail_refresh = 0

		self.flow = [] #the flow data computed by getvector

		self.n = 0

		#self.image = []

		self.linearfly()


	def takeimage(self, img): #[front camera image from subscriber] runs image processing, and feeds the resulting pose data into the navigation algorithm

		self.image = img



	

	def linearfly(self): #takes drone off and flies it in a line, used to get it moving so that it can reliably triangulate

		time.sleep(1) #needed for takeoff publisher to initialize
		
		self.pub_takeoff.publish(Empty())

		time.sleep(2) #lets drone get into the air

		twist = Twist()
		twist.linear.x = .1; twist.linear.y = 0; twist.linear.z = 0; twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		self.pub_twist.publish(twist) 
		print "cmdsent"
		time.sleep(1) #builds up a steady forward speed for accurate triangulation

		#print "done"
	

	def odometry(self, data): #[odometry from subscriber]  the odometry is itself optical flow based
		#stores odometry data to integrate with vision processing
		print data
		#print data.twist.twist.linear.y

		self.v = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.angular.x] #angular is always 0 regardless of whether drone is flying
	
		#if (self.initframe): #initializes points once when program starts

		#	self.linearfly()
		#	self.initframe = False
	
		self.processimage2(self.image, self.v)


	def processimage2(self, imgdrone, data): #performs optical flow and point set maintainence tasks

		imgbgr = self.bridge.imgmsg_to_cv2(imgdrone, "bgr8") #converts from drone image to bgr8 for opencv

		cv2.circle(imgbgr, (int(self.ctrx+100*data[1]),int(self.ctry+100*data[0])), abs(int(data[2])), (127,127,200), 10)

		#cv2.circle(imgbgr, (int(self.ctrx+-.5*v[1]),self.ctry), 20, color, 10)

		twist = Twist()
		twist.linear.x = .1; twist.linear.y = -0.5*data[1]; twist.linear.z = 0; twist.angular.x = -data[2]; twist.angular.y = 0; twist.angular.z = 0
		self.pub_twist.publish(twist) 

		#print p1[5][0][1] #the zero in the middle is required because the array is nominally 3 dimensional but one dimension has 0 thickness

		imgdrone = self.bridge.cv2_to_imgmsg(imgbgr, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image.publish(imgdrone)

		#imgdrone2 = self.bridge.cv2_to_imgmsg(imgout, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		#self.pub_image2.publish(imgdrone2)


if __name__=="__main__":

	rospy.init_node('Hoop_finder')
        
	node = Hoop_finder()

	rospy.spin()


