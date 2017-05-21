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
import tf

class Hoop_finder:

	def __init__(self):

		self.sub_image = rospy.Subscriber("/ardrone/front/image_raw", Image, self.takeimage, queue_size=1) #gets front camera raw image
		self.sub_odometry = rospy.Subscriber("/ardrone/odometry", Odometry, self.odometry, queue_size=1) #gets odometry data
		
		self.pub_image = rospy.Publisher("~detection_image", Image, queue_size=1) 
		self.pub_image2 = rospy.Publisher("~detection_image2", Image, queue_size=1)
		#publish the drone camera image, with feature points and visual debugging images superimposed

		self.pub_land = rospy.Publisher('/ardrone/land',Empty,queue_size=1) #lands drone
		self.pub_takeoff = rospy.Publisher('/ardrone/takeoff',Empty,queue_size=1) #makes drone take off	
		self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 1) #publishes commands to drone

		self.bridge = CvBridge() #used to convert between opencv's and the drone's image formats

		self.imagex = 640
		self.imagey = 360 
		#dimensions of the image, in pixels

		self.ctrx = int(self.imagex/2.0)
		self.ctry = int(self.imagey/2.0) 
		#coordinates of image center, in pixels

		self.old_gray = np.zeros((self.imagex, self.imagey), dtype = "uint8") #grayscale version of previous image used to compute flow
		self.mask = np.zeros((self.imagex, self.imagey, 3), dtype = "uint8") #stores point trails

		self.maxcorners = 400 #maximum number of feature points used

		self.p0 = np.zeros((self.maxcorners,1,2), dtype = "float32") #previous point set used to compute flow

		self.rollavglength = 10 
		self.rralen = 50
		#the numbers of values to use for rolling averages; long arrays are less sensitive and lag more, but resist noise better
		self.rollingavgdata = np.zeros((3,100), dtype = "float32") 
		self.rra = np.zeros((self.rralen), dtype = "float32")
		#store data values over time to compute rolling averages

		self.v = np.zeros((2), dtype = "float32") #horizontal velocity from bottom camera odometry

		self.movedpts = np.zeros((self.maxcorners,2), dtype = "uint8") #initializes the tracking array for regenerated points

		# Create some random colors for each trail
		self.color = np.random.randint(0,255,(self.maxcorners,3))
		# Parameters for lucas kanade optical flow, used repeatedly
		self.lk_params = dict(winSize  = (15,15),maxLevel = 5,criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
		# params for ShiTomasi corner detection
		self.feature_params = dict(maxCorners = self.maxcorners,qualityLevel = 0.2,minDistance = 7,blockSize = 7)
		#a quality level of .3 seemed worse

		self.initframe = True #used to initialize point set once on the first camera frame	
		self.initframe2 = True #used to initialize point set once on the first camera frame	

		self.trail_refresh = 0 #a counter used to periodically clear trails

		self.flow = [] #the flow data computed by getvector

		self.fast = cv2.FastFeatureDetector() #an alternative to Shi-Tomasi corner detection

		self.n = 0 #a counter used to control the frequency of point triangulation so that their actual movement overwhelms the noise

		self.angles_old = np.zeros((2,self.maxcorners), dtype = "float32") #an angle buffer used to compute anglar velocity of feature points

		self.deltas = [] #feature point angular velocities
		self.positions = [] #triangulated feature point positions
		self.realigned = []

		self.m = 0 #hall regression slope
		self.b = 0 #hall regression y intercept

		self.x0 = 0
		self.y0 = 0
		self.t0 = 0


	def takeimage(self, img): #[front camera image from subscriber] runs image processing, and feeds the resulting pose data into the navigation algorithm

		if (self.initframe): #initializes points once when program starts

			self.initflow(img)
			self.linearfly()
			
			self.initframe = False


		self.processimage2(img)	


	def odometry(self, data): #[odometry from subscriber]  the odometry is itself optical flow based
		#stores odometry data to integrate with vision processing
		#print data
		#print data.twist.twist.linear.y

		quaternion = data.pose.pose.orientation
		q2 = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
		euler = tf.transformations.euler_from_quaternion(q2)
		#print euler
		self.v = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.pose.pose.position.x, data.pose.pose.position.y, euler[0]]
		#print self.v

		if (self.initframe2): #initializes points once when program starts

			self.x0 = self.v[2]
			self.y0 = self.v[3]
			self.t0 = self.v[4]

			#print self.x0, self.y0, self.t0
			self.initframe2 = False
		

	def linearfly(self): #takes drone off and flies it in a line, used to get it moving so that it can reliably triangulate

		time.sleep(1) #needed for takeoff publisher to initialize
		
		self.pub_takeoff.publish(Empty())#TODO reenable

		time.sleep(2) #lets drone get into the air

		twist = Twist()
		twist.linear.x = .1; twist.linear.y = 0; twist.linear.z = 0; twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		#self.pub_twist.publish(twist)
		print "cmdsent"
		time.sleep(1) #builds up a steady forward speed for accurate triangulation

		#print "done"


	#########################################################################################################################

	def update_roll_avg(self, newdata): #[x, y, radial] <-- flow values	adds a new data set to the rolling average array, and removes the oldest one
		
		for i in range(1, self.rollavglength): #shifts data

			self.rollingavgdata[0][self.rollavglength-i] = self.rollingavgdata[0][self.rollavglength-i-1]
			self.rollingavgdata[1][self.rollavglength-i] = self.rollingavgdata[1][self.rollavglength-i-1]
			self.rollingavgdata[2][self.rollavglength-i] = self.rollingavgdata[2][self.rollavglength-i-1]

		self.rollingavgdata[0][0] = newdata[0]
		self.rollingavgdata[1][0] = newdata[1] #adds new data
		self.rollingavgdata[2][0] = newdata[2]

		for i in range(1, self.rralen): #shifts data

			self.rra[self.rralen-i] = self.rra[self.rralen-i-1]

		self.rra[0] = newdata[3]


	def get_roll_avg(self): #returns the means of the rolling average arrays

		sums = [0,0,0,0]

		for i in range(1, self.rollavglength): #sums data

			sums[0] += self.rollingavgdata[0][i]
			sums[1] += self.rollingavgdata[1][i]
			sums[2] += self.rollingavgdata[2][i]

		for i in range(1, self.rralen):

			sums[3] += self.rra[i]


		return [sums[0]/self.rollavglength, sums[1]/self.rollavglength, sums[2]/self.rollavglength, sums[3]/self.rralen] #calculates and returns averages

	####################################################################################################################

	def linreg(self, vals): #[points locations] performs linear regression of feature points for which positions could be triangulated to figure out the drone's angle in the hall

		xvals = vals[0]
		yvals = vals[1]

		xnonzero = xvals[xvals!= 0]
		ynonzero = yvals[yvals!= 0]

		if len(xnonzero) != 0: #eliminates empty array issues when the drone is landed and no odometry is available

			m,b = np.polyfit(xnonzero, ynonzero, 1)

		else:

			m,b = 0,0

		return m,b


	def getangles(self, points): #[feature points] returns an array with the x and y angles of all the points

		z = 409.581 #camera dependent constant, with dimensions of pixels

		#range = +- 38deg, the current camera model looks decent, z is computed as (pixel distance from image center)*cot(angle of that point obtained from protractor)

		angles = np.zeros((2, len(points)), dtype = "float32")

		for i in range(0, len(points)):
			
			if self.movedpts[i][0] == 0 and self.movedpts[i][1] == 0: #omits regenerated points

				#r = np.sqrt(points[i][0][0]*points[i][0][0] + points[i][0][1]*points[i][0][1])

				angles[1][i] = np.arctan((points[i][0][1]-self.ctry)/z)
				angles[0][i] = np.arctan((points[i][0][0]-self.ctrx)/z)
				#print angles[0][i]*57 #this produces a very reasonable set of angles, so it probably works properly		
		return angles				
	

	def getdeltas(self, angles1, angles2): #[old angles, new angles] returns an array of the change in angle of each point

		d_angles = np.zeros((2,self.maxcorners), dtype = "float32")

		for i in range(0, len(angles2[0])):
			
			if self.movedpts[i][0] == 0 and self.movedpts[i][1] == 0: #omits regenerated points

				d_angles[0][i] = angles2[0][i] - angles1[0][i]
				d_angles[1][i] = angles2[1][i] - angles1[1][i]
				#print d_angles[0][i]
		return d_angles


	def getlocations(self, angles, deltas, velocity): #combines feature point position and motion with ground odometry to triangulate the points

		locations = np.zeros((2,self.maxcorners), dtype = "float32")

		radii = np.zeros((self.maxcorners), dtype = "float32")

		for i in range(0, len(angles[0])):
			
			if self.movedpts[i][0] == 0 and self.movedpts[i][1] == 0: #omits regenerated points

				if deltas[0][i] != 0 and velocity[1] != 0 and velocity[0] != 0:

					distance = abs((velocity[1]-np.tan(angles[0][i])*velocity[0])*np.cos(angles[0][i])/deltas[0][i])
					
					if distance < 64:

						locations[0][i] = distance*np.sin(angles[0][i])  #lateral location
						locations[1][i] = distance*np.cos(angles[0][i])  #axial location
						radii[i] = distance
					else:

						locations[0][i] = 0
						locations[1][i] = 0
						radii[i] = 0

					#print locations[0][i], locations[1][i]
				
				else:

					locations[0][i] = 0
					locations[1][i] = 0
					radii[i] = 0

		return locations, radii


	def rotate(self, angle, points):

		output = np.zeros_like(points)

		for i in range(0, len(points[0])):

			output[0][i] = points[0][i]*np.cos(angle)+points[1][i]*np.sin(angle)
			output[1][i] = points[1][i]*np.cos(angle)+points[0][i]*np.sin(angle)

		return output


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

			newpts = cv2.goodFeaturesToTrack(gray, mask = None, **fp3) #generates a batch of as many points as need regeneration

			c = 0

			l = len(newpts) 	

			for x in range(0, len(points)):

				if self.movedpts[x][0] == 1 or self.movedpts[x][1] == 1:

					points[x] = newpts[c] #replaces points flagged for regeneration

					c+=1
					if c==l:
						break #terminates once all new points have been used
				
		
	def initflow(self, imgdrone1): #generates feature point array for LK optical flow
	
		#converts from drone image to bgr8 for opencv
		imgbgr = self.bridge.imgmsg_to_cv2(imgdrone1, "bgr8")
		# Creates a grayscale version of the camera image
		self.old_gray = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY)
		# Take first frame and find corners in it

		self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)

		self.mask = np.zeros_like(imgbgr)


	def processimage2(self, imgdrone1): #performs optical flow and point set maintainence tasks

		imgbgr = self.bridge.imgmsg_to_cv2(imgdrone1, "bgr8") #converts from drone image to bgr8 for opencv
		frame_gray = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY) # Creates a grayscale version of the camera image

		# calculates optical flow, p1 is an array of the feature points in their new positions
		p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)

		interval = 10.0

		self.regenedgepoints(p1)

		self.regenbadpoints(p1, st)


		if round(self.n/interval) == self.n/interval:

			angles2 = self.getangles(p1)
			self.deltas = self.getdeltas(self.angles_old, angles2)
			self.angles_old = angles2

			self.positions, distances = self.getlocations(angles2, self.deltas, self.v)
			self.m,self.b = self.linreg(self.positions)			

			self.regenall(p1, frame_gray)

			self.realigned = self.rotate(-self.v[4]+self.t0, self.positions)

			twist = Twist()

			if self.m != 0:

				#print math.atan(1/self.m)
				
				twist.linear.x = .1; twist.linear.y = 0; twist.linear.z = 0; twist.angular.x = -5*math.atan(1/self.m); twist.angular.y = 0; twist.angular.z = 0

			else:

				#print "no data"

				twist.linear.x = .1; twist.linear.y = 0; twist.linear.z = 0; twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0


			#self.pub_twist.publish(twist) 

		self.n+=1		

		

		self.old_gray = frame_gray.copy()

		
		good_new = p1#[st==1]
		good_old = self.p0#[st==1]
		
		if self.trail_refresh == 100:
			#self.mask = np.zeros_like(imgbgr) #TODO reenable when needed
			self.trail_refresh = 0

		#frame_gray2 = np.zeros_like(imgbgr) #provides a black background
		frame_gray2 = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)

		#draws the tracks to show point motion
    		for i,(new,old) in enumerate(zip(good_new,good_old)):
			if self.movedpts[i][0] == 0 and self.movedpts[i][1] == 0:
        			a,b = new.ravel()
        			c,d = old.ravel()
        			#mask = cv2.line(self.mask, (a,b),(c,d), (127,127+30000*self.deltas[0][i],100), 2)#127+100*angles2[0][i]
				#print positions[i][1]

				
				

				frame = cv2.circle(frame_gray2,(a,b),5,(127,int(127+5000*self.deltas[0][i]),int(127+0*self.positions[1][i])),-1)#color constants 5,5
        			cv2.circle(self.mask,(self.ctrx+int(5*(self.positions[0][i]+self.v[3]-self.y0)),self.imagey-int(5*(self.positions[1][i]+self.v[2]-self.x0))),1,(27,27,227),-1)#self.color[i].tolist()

		self.trail_refresh+=1

		cv2.circle(self.mask, (int(20*(self.v[3]-self.y0)+self.ctrx), int(20*(self.v[2]-self.x0)+self.imagey)), 1, (0,127,0), 2)

		#cv2.circle(self.mask,(self.ctrx+int(100*np.sin(self.v[4])), self.ctry+int(100*np.cos(self.v[4]))), 2, (127,0,0), 2)  

		cv2.line(self.mask, (self.ctrx, self.imagey), (self.ctrx+281, 0), (255,255,255))
		cv2.line(self.mask, (self.ctrx, self.imagey), (self.ctrx-281, 0), (255,255,255))

    		imgout = cv2.add(imgbgr, self.mask)

		#updates the previous frame, points, and rolling average

   		self.p0 = p1.copy()
		#self.update_roll_avg([self.flow[0], self.flow[1], self.flow[2], 0)
		
		#print self.v

		

		#cv2.circle(imgbgr, ctr, abs(int(1000*flow[2])), color, 10)
		#cv2.circle(imgbgr, (int(-self.ctrx*flow[3]*4/3.141592), self.ctry), 30, (100,100,50), 10) #*4/3.14159265358979

		#cv2.circle(frame_gray2, (int(self.ctrx-200*self.v[1]), int(self.ctry-200*self.v[0])), 30, (100,90,100), 10)

		#cv2.circle(frame_gray2, (int(self.ctrx), int(self.imagey)), 7, (0,0,0), 2)
		
		#if self.m != 0:
			#print math.atan(1/self.m)

			#cv2.line(frame_gray2, (self.ctrx, self.imagey-int(self.b)*5), (self.ctrx+np.sign(int(self.m))*200, self.imagey-abs(int(self.m*200))-int(self.b)*5), (27,27,227))

			#cv2.circle(frame_gray2, (int(self.ctrx-4*math.atan(1/self.m)), int(self.ctry)), 10, (100,190,100), 5)

		imgdrone = self.bridge.cv2_to_imgmsg(frame_gray2, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image.publish(imgdrone)

		imgdrone2 = self.bridge.cv2_to_imgmsg(imgout, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image2.publish(imgdrone2)


if __name__=="__main__":

	rospy.init_node('Hoop_finder')
        
	node = Hoop_finder()

	rospy.spin()


