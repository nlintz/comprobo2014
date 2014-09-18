#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

class FiniteStateMachine(object):
	def __init__(self, wallFollower, obstacleAvoider):
		self._wallFollower = wallFollower
		self._obstacleAvoider = obstacleAvoider
		self._scanSubscriber = rospy.Subscriber('scan', LaserScan, self._onScan)

	def _onScan(self, msg):
		if min(msg.ranges) < .5:
			self._wallFollower.run()
			self._obstacleAvoider.stop()
		else:
			self._obstacleAvoider.run()
			self._wallFollower.stop()

if __name__ == "__main__":
	fsm = FiniteStateMachine()
	try:
		fsm.run()
	except rospy.ROSInterruptException:pass