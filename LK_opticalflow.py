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

		self.old_gray = np.zeros((360, 640), dtype = "uint8")
		self.mask = np.zeros((360, 640, 3), dtype = "uint8")

		self.p0 = []

		# Create some random colors for each trail
		self.color = np.random.randint(0,255,(100,3))
		# Parameters for lucas kanade optical flow, used repeatedly
		self.lk_params = dict(winSize  = (15,15),maxLevel = 2,criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
		# params for ShiTomasi corner detection, used once
		self.feature_params = dict(maxCorners = 100,qualityLevel = 0.3,minDistance = 7,blockSize = 7)
		self.feature_params2 = dict(maxCorners = 1,qualityLevel = 0.3,minDistance = 7,blockSize = 7)

		self.initframe = True		
		self.n = 0

	def takeimage(self, img): #runs image processing to find hula hoop, and feeds the resulting drone and hoop pose data into the navigation algorithm
		if (self.initframe): 

			self.initflow(img)
			self.initframe = False

		self.processimage2(img)
			
		#self.hoopnav(posedata)	
		##self.n+=1
	
	
	def odometry(self, data): #unused; the odometry is itself optical flow based
		#stores odometry data to integrate with vision processing
		#print data
		#print data.twist.twist.linear.y
		#self.vy = data.twist.twist.linear.y
		a = 1 #boilerplate code for spacing
				
	def getvector(self, points1, points2, updated):

		totalx = 0 #turns into average x/y
		totaly = 0

		totalrad = 0

		pairs = 0

		for x in range(0, len(points1)):

			if updated[x]==0:

				x = points2[x][0][1]-points1[x][0][1]
				y = points2[x][0][0]-points1[x][0][0]

				totalx+=x
				totaly+=y

				totalrad += x*(points2[x][0][1]-180) + y*(points2[x][0][0]-320) #the raw dot product is weighted in favor of points near the edges; idk if this is desirable, I should do some geometry to try to figure out what if any weighting is optimal

		print totalx/len(points1)
		print totaly/len(points1)
		print totalrad/len(points1)
		

	def regenpoints(self, points, gray, updated): #regenerates points which have reached the edges of the frame, and marks these points in an array

		borderwidth = 10
		imagex = 360
		imagey = 640

		upperx = imagex-borderwidth
		uppery = imagey-borderwidth

		for p in points:

			if p[0][1] > upperx or p[0][1] < borderwidth or p[0][0] > uppery or p[0][0] < borderwidth:

				p = cv2.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params2)[0]

		return points


	def initflow(self, imgdrone1): #generates feature point array for LK optical flow
	
		# Take first frame and find corners in it
		imgbgr = self.bridge.imgmsg_to_cv2(imgdrone1, "bgr8")
		self.old_gray = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY)
		self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)
		# Create a mask image for drawing purposes
		self.mask = np.zeros_like(imgbgr)

	def processimage2(self, imgdrone1):	

		imgbgr = self.bridge.imgmsg_to_cv2(imgdrone1, "bgr8")
		frame_gray = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY)
		# calculate optical flow
		p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)
		# Select good points

		movedpts = np.zeros(len(self.p0), dtype = "uint8")

		p1 = self.regenpoints(p1, frame_gray, movedpts)

		self.getvector(self.p0, p1, movedpts)

		good_new = p1[st==1]
		good_old = self.p0[st==1]
		# draw the tracks
    		for i,(new,old) in enumerate(zip(good_new,good_old)):
        		a,b = new.ravel()
        		c,d = old.ravel()
        		mask = cv2.line(self.mask, (a,b),(c,d), self.color[i].tolist(), 2)
        		frame = cv2.circle(imgbgr,(a,b),5,self.color[i].tolist(),-1)
    		imgout = cv2.add(imgbgr,self.mask)

		# Now update the previous frame and previous points
    		self.old_gray = frame_gray.copy()
   		self.p0 = good_new.reshape(-1,1,2)



		#print p1[5][0][1] #the zero in the middle is required because the array is nominally 3 dimensional but one dimension has 0 thickness

		imgdrone = self.bridge.cv2_to_imgmsg(imgbgr, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image.publish(imgdrone)

		imgdrone2 = self.bridge.cv2_to_imgmsg(imgout, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image2.publish(imgdrone2)


if __name__=="__main__":
	rospy.init_node('Hoop_finder')
        
	node = Hoop_finder()

	rospy.spin()
