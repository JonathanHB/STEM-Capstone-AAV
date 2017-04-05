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

		self.sub_image = rospy.Subscriber("/ardrone/image_raw", Image, self.processimage1, queue_size=1)
		self.pub_image = rospy.Publisher("~detection_image", Image, queue_size=1)
		self.pub_image2 = rospy.Publisher("~detection_image2", Image, queue_size=1)
		self.bridge = CvBridge()
		

	def processimage1(self, imgdrone1):
		
		#print "image recieved"

		yellow = [np.array(x, np.uint8) for x in [[25,100,100], [35, 255, 255]]] #the hsv filter used to detect yellow color by cv2.inRange
		#ranges: 0-18-,0-255,0-255
		erosion = (5,5) #values used for erosion by cv2.erode
		
		imgrgb = self.bridge.imgmsg_to_cv2(imgdrone1, "bgr8") #converts drone's raw_image to bgr8 used by opencv
		imghsv = cv2.cvtColor(imgrgb, cv2.COLOR_BGR2HSV) #converts hsv to rgb color

		imgyellow = cv2.inRange(imghsv, yellow[0], yellow[1]) #filter the image for yellow, returns 8UC1 binary image
		
		erosion = (5,5)
		imgyellow = cv2.erode(imgyellow, erosion, iterations = 3)
		dilation = (5,5)
		imgyellow = cv2.dilate(imgyellow, dilation, iterations = 5) #erodes, then dilates the image to remove noise points

		self.pub_image2.publish(self.bridge.cv2_to_imgmsg(imgyellow, "8UC1")) #publish filtered binary image	
         
		filtered_contours = [] #creates array to store contours
		contours, hierarchy = cv2.findContours(\
                imgyellow,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE) #finds image contours
		contour_area = [ (cv2.contourArea(c), (c) ) for c in contours] #pairs contours with size data
		contour_area = sorted(contour_area,reverse=True, key=lambda x: x[0]) #sorts contours by size

		imgcont = imgrgb #saves rgb image without drawn elements

		cv2.drawContours(imgcont, contours, -1, (0,255,0), 3) #draws all contours

		#a,b = contour_area[0] #splits contours (numpy arrays) from area data, doesn't need to be an independent step
		#c,d = contour_area[1]
		
		stacked = np.vstack((contour_area[0][1],contour_area[1][1])) #combines the two largest contours to find hoop
		ellipse = cv2.fitEllipse(stacked) #fits the ellipse to the combined contour
		
		#ellipse data: 0=center, 1=size, 2=angle	

		#elldata = cv2.moments(stacked)
		#cx = int(elldata['m10']/elldata['m00'])
		#cy = int(elldata['m01']/elldata['m00']) #gets center of contour, not ellipse, unstable

	#	cv2.circle(imgcont, (int(round(ellipse[0][0])), int(round(ellipse[0][1]))) , 30, (200,100,0)) #draws circle at ellipse center
		
		cv2.rectangle(imgcont, (int(round(ellipse[0][0]+ellipse[1][0]*.3)), int(round(ellipse[0][1]+ellipse[1][1]*.3))), (int(round(ellipse[0][0]-ellipse[1][0]*.3)), int(round(ellipse[0][1]-ellipse[1][1]*.3))), (200,100,0)) #draws a rectangle at ellipse center with dimensions proportional to those of ellipse

		#print math.acos(ellipse[1][0]/ellipse[1][1]) #computes and prints hoop angle

		#cv2.drawContours(imgcont, b, -1, (0,0,255), 3)
		#cv2.drawContours(imgcont, d, -1, (255,0,255), 3)
    		cv2.ellipse(imgcont,ellipse,(255,0,0),2) #draws ellipse

		imgdrone2 = self.bridge.cv2_to_imgmsg(imgcont, "8UC3") #converts opencv's bgr8 back to the drone's raw_image for rviz use, converts both hsv and rgb to rviz-readable form

		self.pub_image.publish(imgdrone2)

if __name__=="__main__":
	rospy.init_node('Hoop_finder')
        
	node = Hoop_finder()

	rospy.spin()
