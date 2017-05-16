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

		self.maxcorners = 200 #maximum number of points used

		self.p0 = np.zeros((2,0,self.maxcorners), dtype = "float32") #previous point set used to compute flow

		self.lastangles = [] #used to compute d-theta

		self.rollavglength = 10 #the number of values to use for the rolling average; long arrays are less sensitive and lag more, but resist noise better
		self.rollingavgdata = np.zeros((3,100), dtype = "float32") #stores the last [rollingavglength] flow values to compute mean
		self.v = [] #horizontal velocity from bottom camera

		#initializes the tracking array for regenerated points
		self.movedpts = np.zeros((self.maxcorners,2), dtype = "uint8")

		# Create some random colors for each trail
		self.color = np.random.randint(0,255,(self.maxcorners,3))
		# Parameters for lucas kanade optical flow, used repeatedly
		self.lk_params = dict(winSize  = (15,15),maxLevel = 2,criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
		# params for ShiTomasi corner detection
		self.feature_params = dict(maxCorners = self.maxcorners,qualityLevel = 0.2,minDistance = 7,blockSize = 7)

		self.initframe = True #used to initialize point set once on the first camera frame	

		self.trail_refresh = 0

		self.flow = [] #the flow data computed by getvector

		self.n = 0

		self.lastradial = 0


	def takeimage(self, img): #[front camera image from subscriber] runs image processing, and feeds the resulting pose data into the navigation algorithm

		if (self.initframe): #initializes points once when program starts

			self.initflow(img)
			self.linearfly()
			self.initframe = False


		self.processimage2(img)
	

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
		#print data
		#print data.twist.twist.linear.y

		self.v = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.angular.x]
	

	def getvector(self, points1, points2): #[old point positions, new point positions, indices of regenerated points] obtains the average x, y, and radial in/out motion of points

		totalx = 0 #turns into average x/y
		totaly = 0

		rads = np.zeros(self.maxcorners, dtype = "float32")

		totalrad = 0 #turns into average radial motion

		pixperrad = 482 #use assumes azimuthal angular velocity is in rad/sec

		if self.v[0] != 0:
			dctrx = np.arctan(self.v[1]/self.v[0])
			#print dctrx
		else:
			dctrx = 0

		for i in range(0, len(points1)):
			
			if self.movedpts[i][0] == 0 and self.movedpts[i][1] == 0: #omits regenerated points for which flow would be computed between old and new positions

				x = points2[i][0][0]-points1[i][0][0]
				y = points2[i][0][1]-points1[i][0][1]

				totalx+=x
				totaly+=y

				px = points2[i][0][0]-self.ctrx-dctrx*pixperrad
				py = points2[i][0][1]-self.ctry

				rads[i] = (x*px + y*py)/(px*px + py*py+1) #np.sqrt() #this dot product should be fairly evenly weighted 
				totalrad += rads[i]

		self.flow = [totalx/len(points1), totaly/len(points1), totalrad/len(points1)] #returns average flow values
		
		return dctrx*pixperrad, self.v[2]*pixperrad, rads
		

	def regenbadpoints(self, points, quality): #[point locations, list of points with and without flow] flags and deflags points for which flow cannot be found

		for x in range(0, len(points)): 

			if quality[x] == 1:
				
				self.movedpts[x][0] = 0

			else:

				self.movedpts[x][0] = 1


	def regenedgepoints(self, points): #[point locations] flags and deflags points which are too close to the edge

		borderwidth = 10 #how close (in pixels) points have to be to the border to be regenerated

		uppery = self.imagey-borderwidth
		upperx = self.imagex-borderwidth

		for x in range(0, len(points)):

			if points[x][0][1] > uppery or points[x][0][1] < borderwidth or points[x][0][0] > upperx or points[x][0][0] < borderwidth: #checks point position

				self.movedpts[x][1] = 1

			else:

				self.movedpts[x][1] = 0


	def regenall(self, points, gray): #[points, grayscale image used as regeneration argument] regenerates flagged points

		m = 0

		for x in range(0, len(points)): #counts how many points need regeneration

			if self.movedpts[x][0] == 1 or self.movedpts[x][1] == 1:
				
				m+=1
								
		if m != 0:

			fp3 = dict(maxCorners = m, qualityLevel = 0.2,minDistance = 7,blockSize = 7)

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

		interval = 10.0

		if round(self.n/interval) == self.n/interval:

			self.a, b, self.rvels = self.getvector(self.p0, p1)

			points = self.regenall(p1, frame_gray)

			print self.flow[2]

			radial = self.flow[2]/(abs(self.v[0])+1)

			deriv = radial-self.lastradial
			kp = -.8 #-.04
			kd = .01

			twist = Twist()
			twist.linear.x = .1+kp*radial+kd*deriv; twist.linear.y = 0; twist.linear.z = 0; twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
			self.pub_twist.publish(twist) 

			self.lastradial = radial
		
		self.n += 1

		self.regenedgepoints(p1)

		self.regenbadpoints(p1, st) #this plus the block of 4 methods above contribute 1.5s of lag

		
		good_new = p1#[st==1]
		good_old = self.p0#[st==1]
		
		if self.trail_refresh == 50:
			self.mask = np.zeros_like(imgbgr)
			self.trail_refresh = 0

		demobg = np.zeros_like(imgbgr)
		
		#draws the tracks to show point motion
    		for i,(new,old) in enumerate(zip(good_new,good_old)):
			if self.movedpts[i][0] == 0 and self.movedpts[i][1] == 0:

        			a,b = new.ravel()
        			c,d = old.ravel()
        			mask = cv2.line(self.mask, (a,b),(c,d), (100,127,100), 2)#127+100*angles2[0][i]
        			frame = cv2.circle(imgbgr,(a,b),5,(100,127+30*self.rvels[i],100),-1)#self.color[i].tolist()
				#if angles2[0][i] > 0 and p1[i][0][0] < self.ctrx:

				#	print "---"

		self.trail_refresh+=1

    		imgout = cv2.add(imgbgr, self.mask)

		#updates the previous frame, points, and rolling average
    		self.old_gray = frame_gray.copy()
   		self.p0 = p1.copy()

		#draws a circle onto the camera image based on flow vector for visual debugging

		flow = self.flow
		
		ctr = (self.ctrx+int(9*flow[0]), self.ctry+int(9*flow[1]))

		color = ()
		if flow[2]>=0:
			color = (100,20,100)
		else:
			color = (100,200,100)
			#print "reverse"


		cv2.circle(imgbgr, ctr, abs(int(3000*flow[2])), color, 10)

		cv2.circle(imgbgr, (int(self.ctrx-self.a),self.ctry), 20, color, 10)



		#print p1[5][0][1] #the zero in the middle is required because the array is nominally 3 dimensional but one dimension has 0 thickness

		imgdrone = self.bridge.cv2_to_imgmsg(imgbgr, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image.publish(imgdrone)

		imgdrone2 = self.bridge.cv2_to_imgmsg(imgout, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image2.publish(imgdrone2)


if __name__=="__main__":

	rospy.init_node('Hoop_finder')
        
	node = Hoop_finder()

	rospy.spin()


