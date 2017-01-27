#!/usr/bin/env/python
import os
import rospy
import signal
import Popen
import sys
import subprocess

class generic_py_test():
    def __init__(self):
	Popen.send_signal(signal.SIGINT)

    def updown(self):
	print "started"
	self.takeoff()
	print "takeoff"
    	rospy.sleep(5)
	print "landing"
    	self.land()
	print "landed"

    def takeoff(self):
	os.system("rostopic pub /ardrone/takeoff std_msgs/Empty")
	os.system("\x03")

    def land(self):
        os.system("rostopic pub /ardrone/land std_msgs/Empty")

if __name__ == '__main__':
    rospy.init_node('generic_py_test')
    node = generic_py_test()

