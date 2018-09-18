#!/usr/bin/env python

from std_srvs.srv import Empty
import rospy
from duckietown_msgs.msg import FSMState
import numpy as np

class Path_planning():
	def __init__(self):
		self.fsm_sub = rospy.Subscriber("/pbody/fsm_node/mode", FSMState, self.fsm_mode)
<<<<<<< HEAD
		self.path = [2,2,2,2,2,2,2]
=======
		self.path = [2,1,2,0,0,0,1,0,0,1,0,0,2,]
>>>>>>> 345f8d17d5eedde90006bfa3d33e67137ab44408
		self.count = 0
	def fsm_mode(self, msg):
		if msg.state == "INTERSECTION_CONTROL":
			print "Intersection callback"
			self.stop_line = 1
			print "start path planning"        
			self.assign_path()    
	def assign_path(self):
		if self.count < len(self.path):
			control = self.path[self.count]
			self.count+=1
		else:
			control = 3
		rospy.sleep(1)
		if control == 0:
			print "right"
			turn_right = rospy.ServiceProxy('/pbody/open_loop_intersection_control_node/turn_right',Empty)
			turn = turn_right()
			print "r"	
		elif control == 1:
			print "left"
			turn_left = rospy.ServiceProxy('/pbody/open_loop_intersection_control_node/turn_left',Empty)
			turn = turn_left()
			print "l"
		elif control == 2:
			print"forward"
			turn_forward = rospy.ServiceProxy('/pbody/open_loop_intersection_control_node/turn_forward',Empty)
			turn = turn_forward()	
			print "f"



if __name__ == '__main__': 
	rospy.init_node('Path_planning',anonymous=True)
	node = Path_planning()
	rospy.spin()
