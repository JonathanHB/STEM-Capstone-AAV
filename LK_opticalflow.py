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

		self.imagex = 360 #dimensions of the image, in pixels
		self.imagey = 640

		self.ctrx = int(self.imagex/2.0) #coordinates of image center, in pixels
		self.ctry = int(self.imagey/2.0)

		self.old_gray = np.zeros((self.imagex, self.imagey), dtype = "uint8") #grayscale version of previous image used to compute flow
		self.mask = np.zeros((self.imagex, self.imagey, 3), dtype = "uint8") #stores point trails

		self.p0 = [] #previous point set used to compute flow

		maxcorners = 400 #maximum number of points used

		self.rollavglength = 10 #the number of values to use for the rolling average; long arrays are less sensitive and lag more, but resist noise better
		self.rollingavgdata = [[0.0]*self.rollavglength, [0.0]*self.rollavglength, [0.0]*self.rollavglength] #stores the last [rollingavglength] flow values to compute mean
		self.v = [] #horizontal velocity from bottom camera

		#initializes the tracking array for regenerated points
		self.movedpts = np.zeros((maxcorners,2), dtype = "uint8")

		# Create some random colors for each trail
		self.color = np.random.randint(0,255,(maxcorners,3))
		# Parameters for lucas kanade optical flow, used repeatedly
		self.lk_params = dict(winSize  = (15,15),maxLevel = 2,criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
		# params for ShiTomasi corner detection
		self.feature_params = dict(maxCorners = maxcorners,qualityLevel = 0.1,minDistance = 7,blockSize = 7)

		self.initframe = True #used to initialize point set once on the first camera frame	

		self.flow = [] #the flow data computed by getvector


	def takeimage(self, img): #[front camera image from subscriber] runs image processing, and feeds the resulting pose data into the navigation algorithm
		if (self.initframe): #initializes points once when program starts

			self.initflow(img)
			self.initframe = False

		self.processimage2(img)
	
	
	def odometry(self, data): #[odometry from subscriber]  the odometry is itself optical flow based
		#stores odometry data to integrate with vision processing
		#print data
		#print data.twist.twist.linear.y
		self.v = [data.twist.twist.linear.x, data.twist.twist.linear.y]
		

	def update_roll_avg(self, newdata): #[x, y, radial] <-- flow values	adds a new data set to the rolling average array, and removes the oldest one
		
		for i in range(1, self.rollavglength): #shifts data

			self.rollingavgdata[0][self.rollavglength-i] = self.rollingavgdata[0][self.rollavglength-i-1]
			self.rollingavgdata[1][self.rollavglength-i] = self.rollingavgdata[1][self.rollavglength-i-1]
			self.rollingavgdata[2][self.rollavglength-i] = self.rollingavgdata[2][self.rollavglength-i-1]

		self.rollingavgdata[0][0] = newdata[0]
		self.rollingavgdata[1][0] = newdata[1] #adds new data
		self.rollingavgdata[2][0] = newdata[2]
		

	def get_roll_avg(self): #returns the means of the rolling average arrays

		sums = [0,0,0]

		for i in range(1, self.rollavglength): #sums data

			sums[0] += self.rollingavgdata[0][i]
			sums[1] += self.rollingavgdata[1][i]
			sums[2] += self.rollingavgdata[2][i]

		return [sums[0]/self.rollavglength, sums[1]/self.rollavglength, sums[2]/self.rollavglength] #calculates and returns averages
	

	def getangle(points): 

		angles = [[0.0]#TODO finish this at home

		for i in range(0, len(points)):
			
			if self.movedpts[i][0] == 0 and self.movedpts[i][0] == 0: #omits regenerated points

				r = np.sqrt(points[i][0][0]*points[i][0][0] + points[i][0][1]*points[i][0][1])

				c = 1 #camera dependent constant				
	

	def getvector(self, points1, points2): #[old point positions, new point positions, indices of regenerated points] obtains the average x, y, and radial in/out motion of points

		totalx = 0 #turns into average x/y
		totaly = 0

		totalrad = 0 #turns into average radial motion

		pairs = 0

		for i in range(0, len(points1)):
			
			if self.movedpts[i][0] == 0 and self.movedpts[i][0] == 0: #omits regenerated points for which flow would be computed between old and new positions

				x = points2[i][0][1]-points1[i][0][1]
				y = points2[i][0][0]-points1[i][0][0]

				totalx+=x
				totaly+=y

				px = points2[i][0][1]-self.ctrx
				py = points2[i][0][0]-self.ctry

				totalrad += (x*px + y*py)/(px*px+py*py) #the raw dot product is weighted in favor of points near the edges; this is the most computationally efficient way to do it as far as I know, but idk if it is desirable, I should do some geometry to try to figure out what if any weighting is optimal

		self.flow = [totalx/len(points1), totaly/len(points1), totalrad/len(points1)] #returns average flow values

		
	def regenbadpoints(self, points, quality): #[point locations, list of points with and without flow] flags and deflags points for which flow cannot be found

		for x in range(0, len(points)): 

			if quality[x] == 1:
				
				self.movedpts[x][0] = 0

			else:

				self.movedpts[x][0] = 1


	def regenedgepoints(self, points): #[point locations] flags and deflags points which are too close to the edge

		borderwidth = 10 #how close (in pixels) points have to be to the border to be regenerated

		upperx = self.imagex-borderwidth
		uppery = self.imagey-borderwidth

		for x in range(0, len(points)):

			if points[x][0][1] > upperx or points[x][0][1] < borderwidth or points[x][0][0] > uppery or points[x][0][0] < borderwidth: #checks point position

				self.movedpts[x][1] = 1

			else:

				self.movedpts[x][1] = 0


	def regenall(self, points, gray): #[points, grayscale image used as regeneration argument] regenerates flagged points

		m = 0

		for x in range(0, len(points)): #counts how many points need regeneration

			if self.movedpts[x][0] == 1 or self.movedpts[x][1] == 1:
				
				m+=1
								
		if m != 0:

			fp3 = dict(maxCorners = m, qualityLevel = 0.1,minDistance = 7,blockSize = 7)

			newpts = cv2.goodFeaturesToTrack(gray, mask = None, **fp3) #generates a batch of as many points need regeneration

			c = 0

			l = len(newpts) 	

			for x in range(0, len(points)):

				if self.movedpts[x][0] == 1 or self.movedpts[x][1] == 1:
				
					points[x][0] = newpts[c][0] #replaces points flagged for regeneration
					c+=1
					if c==l:
						break #terminates once all new points have been used

			
		return points #returns updated point array
				

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

		# calculates optical flow, p1 is an array of the feature points in their new positions
		p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)
		 
		self.getvector(self.p0, p1)

		self.regenedgepoints(p1)

		self.regenbadpoints(p1, st) 

		points = self.regenall(p1, frame_gray)

		good_new = p1#[st==1]
		good_old = self.p0#[st==1]

		#draws the tracks to show point motion
    		for i,(new,old) in enumerate(zip(good_new,good_old)):
        		a,b = new.ravel()
        		c,d = old.ravel()
        		mask = cv2.line(self.mask, (a,b),(c,d), self.color[i].tolist(), 2)
        		frame = cv2.circle(imgbgr,(a,b),5,self.color[i].tolist(),-1)
    		imgout = cv2.add(imgbgr,self.mask)

		#updates the previous frame, points, and rolling average
    		self.old_gray = frame_gray.copy()
   		self.p0 = p1
		self.update_roll_avg(self.flow)


		#draws a circle onto the camera image based on flow vector for visual debugging
		flow = self.get_roll_avg()

		ctr = (self.ctry+int(9*flow[1]),self.ctrx+int(9*flow[0]))
		ctr = self.ctry+int(9*flow[1]),180
		color = ()
		if flow[2]>=0:
			color = (100,200,0)
		else:
			color = (100,200,100)
			#print "reverse"


		cv2.circle(imgbgr, ctr, abs(int(1000*flow[2])), color, 10)


		#print p1[5][0][1] #the zero in the middle is required because the array is nominally 3 dimensional but one dimension has 0 thickness

		imgdrone = self.bridge.cv2_to_imgmsg(imgbgr, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image.publish(imgdrone)

		imgdrone2 = self.bridge.cv2_to_imgmsg(imgout, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image2.publish(imgdrone2)


if __name__=="__main__":

	rospy.init_node('Hoop_finder')
        
	node = Hoop_finder()

	rospy.spin()


