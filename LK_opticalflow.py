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

		self.imagex = 360
		self.imagey = 640

		self.ctrx = int(self.imagex/2.0)
		self.ctry = int(self.imagey/2.0)

		self.old_gray = np.zeros((self.imagex, self.imagey), dtype = "uint8")
		self.mask = np.zeros((self.imagex, self.imagey, 3), dtype = "uint8")

		self.p0 = []

		maxcorners = 200

		#initializes the tracking array for regenerated points
		self.movedpts = np.zeros((maxcorners,2), dtype = "uint8")

		# Create some random colors for each trail
		self.color = np.random.randint(0,255,(maxcorners,3))
		# Parameters for lucas kanade optical flow, used repeatedly
		self.lk_params = dict(winSize  = (15,15),maxLevel = 2,criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
		# params for ShiTomasi corner detection, used once
		self.feature_params = dict(maxCorners = maxcorners,qualityLevel = 0.1,minDistance = 7,blockSize = 7)
		self.feature_params2 = dict(maxCorners = 1,qualityLevel = 0.1,minDistance = 7,blockSize = 7)

		self.initframe = True		
		self.n = 0

		self.flow = []

	def takeimage(self, img): #[front camera image from subscriber] runs image processing, and feeds the resulting pose data into the navigation algorithm
		if (self.initframe): #initializes points once when program starts

			self.initflow(img)
			self.initframe = False

		self.processimage2(img)
			
		#self.hoopnav(posedata)	
		##self.n+=1
	
	
	def odometry(self, data): #[odometry from subscriber] unused; the odometry is optical flow based
		#stores odometry data to integrate with vision processing
		#print data
		#print data.twist.twist.linear.y
		#self.vy = data.twist.twist.linear.y
		a = 1 #meaningless code to satisfy spacing syntax rules
				
	def getvector(self, points1, points2): #[old point positions, new point positions, indices of regenerated points] obtains the average x, y, and radial in/out motion of points

		totalx = 0 #turns into average x/y
		totaly = 0

		totalrad = 0 #turns into average radial motion

		pairs = 0

		for i in range(0, len(points1)):
			
			if self.movedpts[i][0] == 0 and self.movedpts[i][0] == 0: #omits regenerated points

				x = points2[i][0][1]-points1[i][0][1]
				y = points2[i][0][0]-points1[i][0][0]

				totalx+=x
				totaly+=y

				totalrad += x*(points2[i][0][1]-self.ctry) + y*(points2[i][0][0]-self.ctrx) #the raw dot product is weighted in favor of points near the edges; this is the most computationally efficient way to do it as far as I know, but idk if it is desirable, I should do some geometry to try to figure out what if any weighting is optimal

		self.flow = (totalx/len(points1), totaly/len(points1), totalrad/len(points1)) #these will be changed to return statements and fed into the navigation algorithm once the optical processing is in good working order

		

	def regenbadpoints(self, points, quality):

		for x in range(0, len(points)): 

			if quality[x] == 1:
				
				self.movedpts[x][0] = 0

			else:

				self.movedpts[x][0] = 1

				#points[x] = cv2.goodFeaturesToTrack(gray, mask = None, **self.feature_params2) #regenerates points				

		#return self.movedpts


	def regenedgepoints(self, points): #[point locations, grayscale image, update tracking array] regenerates points which have reached the edges of the frame, and marks these points in an array so that their change to new locations isn't interpereted as motion

		borderwidth = 10 #how close (in pixels) points have to be to the border to be regenerated

		upperx = self.imagex-borderwidth
		uppery = self.imagey-borderwidth

		for x in range(0, len(points)):

			if points[x][0][1] > upperx or points[x][0][1] < borderwidth or points[x][0][0] > uppery or points[x][0][0] < borderwidth: #checks point position

				self.movedpts[x][1] = 1

			else:

				self.movedpts[x][1] = 0

		#return self.movedpts #returns new point array, "self.movedpts" array is modified as a side effect


	def regenall(self, points, gray): #regenerates points flagged as bad

		m = 0
		for x in range(0, len(points)):

			if self.movedpts[x][0] == 1 or self.movedpts[x][1] == 1:
				
				m+=1
				
				
		if m != 0:
			fp3 = dict(maxCorners = m, qualityLevel = 0.1,minDistance = 7,blockSize = 7)
			newpts = cv2.goodFeaturesToTrack(gray, mask = None, **fp3) #regenerates points
			c = 0

			l = len(newpts) 
			#print m	

			for x in range(0, len(points)):

				if self.movedpts[x][0] == 1 or self.movedpts[x][1] == 1:
				
					points[x][0] = newpts[c][0]
					c+=1
					if c==l:
						break

			
		return points 	
				

	def initflow(self, imgdrone1): #generates feature point array for LK optical flow
	
		#converts from drone image to bgr8 for opencv
		imgbgr = self.bridge.imgmsg_to_cv2(imgdrone1, "bgr8")
		# Creates a grayscale version of the camera image
		self.old_gray = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY)
		# Take first frame and find corners in it
		self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)
		# Create a mask image for drawing purposes
		self.mask = np.zeros_like(imgbgr)

	def processimage2(self, imgdrone1): #performs optical flow and point set maintainence tasks

		imgbgr = self.bridge.imgmsg_to_cv2(imgdrone1, "bgr8") #converts from drone image to bgr8 for opencv
		frame_gray = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY) # Creates a grayscale version of the camera image
		# calculate optical flow, p1 is an array of the feature points in their new positions
		p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)
		
		#updates point array with regenerated points
		points = self.regenedgepoints(p1) 

		self.getvector(self.p0, p1)

		#self.movedpts = np.zeros(100, dtype = "uint8")

		self.regenbadpoints(p1, st) 
		#self.movedpts = self.regenbadpoints(self.p0, frame_gray, st, self.movedpts, False)

		points = self.regenall(p1, frame_gray)

		good_new = p1#[st==1]
		good_old = self.p0#[st==1] 
		#this system gradually drains out all the points without replacing them, so I need to replace it with a regeneration function based on the same st flag to keep a good point supply. This will share the movedpts tracking array with regenpoints

		#draws the tracks to show point motion
    		for i,(new,old) in enumerate(zip(good_new,good_old)):
        		a,b = new.ravel()
        		c,d = old.ravel()
        		mask = cv2.line(self.mask, (a,b),(c,d), self.color[i].tolist(), 2)
        		frame = cv2.circle(imgbgr,(a,b),5,self.color[i].tolist(),-1)
    		imgout = cv2.add(imgbgr,self.mask)

		#updates the previous frame and points
    		self.old_gray = frame_gray.copy()
   		self.p0 = p1#good_new.reshape(-1,1,2)


		#ctr = (self.ctry+int(9*self.flow[1]),self.ctrx+int(9*self.flow[0]))
		ctr = 320,180
		cv2.circle(imgbgr, ctr, abs(int(.01*self.flow[2])), (100,200,0))


		#print p1[5][0][1] #the zero in the middle is required because the array is nominally 3 dimensional but one dimension has 0 thickness

		imgdrone = self.bridge.cv2_to_imgmsg(imgbgr, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image.publish(imgdrone)

		imgdrone2 = self.bridge.cv2_to_imgmsg(imgout, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image2.publish(imgdrone2)


if __name__=="__main__":
	rospy.init_node('Hoop_finder')
        
	node = Hoop_finder()

	rospy.spin()
