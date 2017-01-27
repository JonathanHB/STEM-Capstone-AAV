#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
import sys
import threading
import time
import math

class Hoop_finder:

	def __init__(self):

		self.sub_image = rospy.Subscriber("/ardrone/image_raw", Image, self.takeimage, queue_size=1)
		self.sub_image = rospy.Subscriber("/ardrone/odometry", Twist, self.odometry, queue_size=1)
		self.pub_image = rospy.Publisher("~detection_image", Image, queue_size=1)
		self.pub_image2 = rospy.Publisher("~detection_image2", Image, queue_size=1)
		self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		self.bridge = CvBridge()
		
		self.Kvals = ((-1,0,0),(.1,-.01,0),(.08,0,0)) #the azimuthal, axial, and lateral PID constants #y:d = -.005
		self.PDIvals = [[0,0,0],[0,0,0],[0,0,0]] #the errors, derivatives, and integrals for each dimension
		self.params = [[0,.2],[1.9,.05],[.25,.04]] #the objective and maximum values
		self.Ebuffer = [0,0,0]#used for derivative computation, can be declared in hoopnav as long as second derivatives aren't being computed
		
		self.revcount = 0 #used to recheck direction intermittently
		self.direction = -1 #the drone's guess as to the right direction to be going
		self.vy = 0 #odometry y velocity

		#self.ybuffer = 0;
		#testinput = [1,2,3]
		#self.hoopnav(testinput)
		#self.hoopnav([1.3,2,2.7])

	def takeimage(self, img):
		
		posedata = self.processimage1(img)
		self.hoopnav(posedata)	

	def hoopnav(self, pose):
		
		if(pose[0] != -1 or pose[1] != -1 or pose[2] != -1): #if a hoop is actually detected

			#self.ybuffer = self.Ebuffer[2]

			self.Ebuffer = [self.PDIvals[0][0], self.PDIvals[1][0], self.PDIvals[2][0]] #buffers errors for derivative computation

			self.PDIvals[0][0] = pose[0]-self.params[0][0]
			self.PDIvals[1][0] = pose[1]-self.params[1][0] #computes errors
			self.PDIvals[2][0] = pose[2]-self.params[2][0]
	
			self.PDIvals[0][1] = self.PDIvals[0][0]-self.Ebuffer[0]
			self.PDIvals[1][1] = self.PDIvals[1][0]-self.Ebuffer[1] #computes derivatives (and errors in the process)
			self.PDIvals[2][1] = self.PDIvals[2][0]-self.Ebuffer[2]

			self.PDIvals[0][2] += self.PDIvals[0][0]
			self.PDIvals[1][2] += self.PDIvals[1][0] #computes integrals
			self.PDIvals[2][2] += self.PDIvals[2][0]
			
			#second_deriv = self.PDIvals[2][1]-(self.Ebuffer[2]-self.ybuffer)
			#sign = second_deriv/abs(second_deriv)
			#print sign
			#print self.PDIvals[2][1]/abs(self.PDIvals[2][1])

			print self.PDIvals[2][1]
			if (self.revcount*.02).is_integer(): #*.04
					
				self.direction *= -np.sign(self.PDIvals[2][1]) #changes direction if angle is increasing
				print "flip"

			self.revcount+=1

			x,y,z = (0,0,0)
			ro,po,az = (0,0,0)

			az = self.Kvals[0][0]*self.PDIvals[0][0]+self.Kvals[0][1]*self.PDIvals[0][1]+self.Kvals[0][2]*self.PDIvals[0][2]
			x = self.Kvals[1][0]*self.PDIvals[1][0]+self.Kvals[1][1]*self.PDIvals[1][1]+self.Kvals[1][2]*self.PDIvals[1][2]	
			y = self.Kvals[2][0]*self.PDIvals[2][0]+self.Kvals[2][1]*abs(self.PDIvals[2][1])+self.Kvals[2][2]*self.PDIvals[2][2]
			y *= self.direction #doesn't flip flop constantly when direction is positive bcz y is recalculated each time

			#az += -y/pose[1] #w=v/r, I'm pretty sure the negative sign is right, but the equation needs a constant (to be found experimentally) to convert between the drone's angular and linear velocities (the commands sent don't map directly to m/s and rad/s), and the error in r will cause some small issues
	
			if az > self.params[0][1]:
				az = self.params[0][1]
			if az < -self.params[0][1]:
				az = -self.params[0][1]
			
			if x > self.params[1][1]:
				x = self.params[1][1]
			if x < -self.params[1][1]:
				x = -self.params[1][1]            #constrains x,y, and az values to safe ranges

			if y > self.params[2][1]:
				y = self.params[2][1]
			if y < -self.params[2][1]:
				y = -self.params[2][1]

			#x = 0;
			#az = 0;
			#y = 0;

			twist = Twist()
			twist.linear.x = x; twist.linear.y = y; twist.linear.z = z; twist.angular.x = ro; twist.angular.y = po; twist.angular.z = az
			self.pub_twist.publish(twist) 
		
			print self.direction
			print "dir"
			print y
			print "y"
			#print self.PDIvals[1][0]
			#print "x"
			#print self.PDIvals[0][0]
			#print "az"
	
		else:

			twist = Twist()
			twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0; twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
			self.pub_twist.publish(twist) 

	def odometry(self, imudata):

		print imudata

	def processimage1(self, imgdrone1):
		#this method finds the distance and angle of the hoop and the angle of the drone relative to the hoop
		#print "image recieved"

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

		#self.pub_image2.publish(self.bridge.cv2_to_imgmsg(imgyellow, "8UC1")) #publish filtered binary image	
         
		filtered_contours = [] #creates array to store contours
		contours, hierarchy = cv2.findContours(\
                imgyellow,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE) #finds image contours
		contour_area = [ (cv2.contourArea(c), (c) ) for c in contours] #pairs contours with size data
		contour_area = sorted(contour_area,reverse=True, key=lambda x: x[0]) #sorts contours by size

		imgcont = imgrgb #saves rgb image without drawn elements
		
		if(len(contour_area)>1):
			stacked = np.vstack((contour_area[0][1],contour_area[1][1])) #combines the two largest contours to find hoop

			if len(stacked)>100:# and len(contour_area[0][1])>40 and len(contour_area[1][1])>40:
		
				ellipse = cv2.fitEllipse(stacked) #fits the ellipse to the combined contour
		
				#ellipse data: 0=center, 1=size, 2=angle	

				#cv2.circle(imgcont, (int(round(ellipse[0][0])), int(round(ellipse[0][1]))) , 30, (200,100,0)) #draws circle at ellipse center
		
				cv2.drawContours(imgcont, contours, -1, (0,255,0), 3) #draws all contours

				cv2.rectangle(imgcont, (int(round(ellipse[0][0]+ellipse[1][0]*.3)), int(round(ellipse[0][1]+ellipse[1][1]*.3))), (int(round(ellipse[0][0]-ellipse[1][0]*.3)), int(round(ellipse[0][1]-ellipse[1][1]*.3))), (200,100,0)) #draws a rectangle at ellipse center with dimensions proportional to those of ellipse

				cv2.ellipse(imgcont,ellipse,(255,0,0),2) #draws ellipse
			
				#the hoop is ~40" in diameter, reads h= 512 at x = 36 inches .5071 = atan((40/2)/36), .5 = radius of hoop in meters

				hoop_angle = math.acos(ellipse[1][0]/ellipse[1][1]) #computes and hoop angle
				hoop_distance = .5*math.cos(.5071*ellipse[1][1]/512)/math.sin(.5071*ellipse[1][1]/512) #computes distance to the hoop
				drone_angle = .5071*(ellipse[0][0]-300)/512 #computes drone angle	

				print hoop_angle
				print hoop_distance
				print drone_angle
				print "--------------------"
				imgdrone2 = self.bridge.cv2_to_imgmsg(imgcont, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to
				self.pub_image.publish(imgdrone2)

				return [drone_angle, hoop_distance, hoop_angle]
			
		imgdrone2 = self.bridge.cv2_to_imgmsg(imgcont, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		

		self.pub_image.publish(imgdrone2)
		return [-1,-1,-1] #tells navigator method no hoop is detected
		

if __name__=="__main__":
	rospy.init_node('Hoop_finder')
        
	node = Hoop_finder()

	rospy.spin()
