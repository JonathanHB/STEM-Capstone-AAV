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
#from nav_msgs import Odometry

#originally a copy of Mr. Batra's drone_detector program

class Detector:

    YELLOW = [np.array(x, np.uint8) for x in [[25,100,100], [45, 255, 255]] ]

    MAX_DETECTIONS = 1 #makes sure that the drone only reacts to the largest object it sees
    ERODE = (5,5)
    
    desired_image_height = 200 #image height is a proxy for distance: 
	#unlike area, it is independent of the drone's horizontal angle, 
	#but still assumes the drone and object are in the same plane

    def get_filtered_contours(self,img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        hsv_img = cv2.erode(hsv_img, self.ERODE, iterations=10)  #XXX
        frame_threshed = cv2.inRange(hsv_img, self.YELLOW[0], self.YELLOW[1])
        ret,thresh = cv2.threshold(frame_threshed, self.YELLOW[0][0], 255, 0)
         

        filtered_contours = []
        contours, hierarchy = cv2.findContours(\
                thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        contour_area = [ (cv2.contourArea(c), (c) ) for c in contours]
        contour_area = sorted(contour_area,reverse=True, key=lambda x: x[0])

        height,width = img.shape[:2]
        for j, (area,(cnt)) in enumerate(contour_area):
            if j > self.MAX_DETECTIONS: continue
            x,y,w,h = cv2.boundingRect(cnt)
            box = (x,y,w,h)
            d =  0.5*(x-width/2)**2 + (y-height)**2 
            if d < 20000: continue # filter tiny images

            mask = np.zeros(thresh.shape,np.uint8)
            cv2.drawContours(mask,[cnt],0,255,-1)
            mean_val = cv2.mean(img,mask = mask)
            aspect_ratio = float(w)/h
            filtered_contours.append( (cnt, box, d, aspect_ratio, mean_val) )
            #print filtered_contours
        return filtered_contours


    def contour_match(self, img):
        '''
        Returns 1. Image with bounding boxes added
                2. an ObstacleImageDetectionList
        '''

        height,width = img.shape[:2]

        cx, cy, wide, high = (.5,.5,0,self.desired_image_height)

        # get filtered contours
        contours = self.get_filtered_contours(img)
        j = 0
        for (cnt, box, ds, aspect_ratio, mean_color)  in contours:
            j += 1            
            # plot box and label around contour
            x,y,w,h = box
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img,"yellow", (x,y), font, 1,mean_color,4) #labels objects
            cv2.rectangle(img,(x,y),(x+w,y+h), mean_color,2)

            if j == 1 and w>100 and h>100: #gets largest object's position and dimensions
		#as the array is sorted, the largest object is at j=1 (Why not j=0??)
              	cx = float(x + w/2)/width
              	cy = float(y + h/2)/height
		wide = w
		high = h
		cv2.putText(img,"|----------|----------|", (x,y), font, 1,mean_color,4) #labels largest image individually

        return img, cx, cy, wide, high

	
		


class StaticObjectDetectorNode:

    def __init__(self):
        self.name = 'static_object_detector_node'
        
        self.detector = Detector()
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/ardrone/image_raw", Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~detection_image", Image, queue_size=1)

	self.t_e_last = 0

#	self.sub_odometry = rospy.Subscriber("/ardrone/odometry", nav_msgs/Odometry, self.parse_odometry, queue_size=1) ##message type needs to be imported
	
	self.desired_image_height = 200 #image height is a proxy for distance: 
	#unlike area, it is independent of the drone's horizontal angle, 
	#but still assumes the drone and object are in the same plane
	
        #self.pub_twist = rospy.Publisher('/ardrone/cmd_vel', Twist, queue_size = 1)
        self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        
        self.bridge = CvBridge()
        rospy.loginfo("[%s] Initialized." %(self.name))
        self.takeoff = rospy.Publisher('/takeoff',Empty) #doesn't work

    #    self.takeoff.publish(Empty)

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        try:
            image_cv=self.bridge.imgmsg_to_cv2(image_msg,"bgr8")
        except CvBridgeErrer as e:
            print e
        img, cx, cy, dx, dy = self.detector.contour_match(image_cv)
	print dy

        twist = Twist()
        speed = 1 #leave this as 1 and change kp/d/i instead  #these two variables may be removed entirely at some point
        turn = 1 #leave this as 1 and change kp/d/i instead
        x,y,z,th = (0,0,0,0)
	#PID control constants	
	kp = -1	
	kd = 1
	ki = 1 ##integral control unimplemented

	kpt = -.001	
	kdt = .001 #this constant may be way off
	kit = 1 ##integral control unimplemented

	cxlast = .5;

#        if cx > 0.55:  #old bang-bang controller
#           th = -1
#        elif cx < 0.45:
#           th = 1

	th = (cx-.5)*kp #+(cx-cxlast)*kd
	
	t_error = dy-self.desired_image_height
#	print t_error
	x = t_error*kpt + (t_error-self.t_e_last)*kdt
	self.t_e_last = t_error
	print x


        twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
	self.pub_twist.publish(twist)             
	
#	print "s1"
#	rospy.sleep(.3)
#	print "s2"

        height,width = img.shape[:2]
        print "center x: " + str(cx) + " center y: " + str(cy)
        #print "height: " + str(height)+"  width: " + str(width)
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

        self.thread_lock.release()

#    def parse_odometry(self, imudata):
	#use dx/dt to get velocity, add corresponding counter-values to the next command
#	placeholder=1 #for python indentation rules
	#print imudata.linear.x 

#    def pidcontrol(self,x,y,w,h):

if __name__=="__main__":
	rospy.init_node('static_object_detector_node')
        #time.sleep(3)
        
	node = StaticObjectDetectorNode()

	rospy.spin()

