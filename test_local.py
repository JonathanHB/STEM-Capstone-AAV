#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
#from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

class testnode(object):
    def __init__(self):

	self.pubLand = rospy.Publisher('ardrone/land',Empty,queue_size=1)
	self.pubTakeoff = rospy.Publisher('ardrone/takeoff',Empty)
	print "pubsinited"
#	self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
	print "subsinited"
	self.updown()
	
    def updown(self):
	self.pubTakeoff.publish(Empty())
        print "takeoff"
	#rospy.sleep(10)
	#self.pubLand.publish(Empty())
        #print "landing"

    def ReceiveNavdata(self,message):
	#x=1
	#x=x+1
	print message

if __name__ == '__main__':
    rospy.init_node('testnode')
    node = testnode()
    rospy.spin()
