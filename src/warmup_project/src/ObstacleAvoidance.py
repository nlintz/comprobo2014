#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

from PIDController import PIDController

class ObstacleAvoider(object):
    """ Class for handling wall following behavior

    """
    def __init__(self):
        self._velocityPublisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self._scanSubscriber = rospy.Subscriber('scan', LaserScan, self._onScan)
        self._velocityMessage = Twist()

    @staticmethod
    def _filterReadings(data, lowerBound=0.0, upperBound=10.0):
        return map(lambda x: x if x > lowerBound and x <= upperBound else float("inf"), data)

    def _sumComponentForces(self, angleDistances):
    	print 'sum components'

    def _onScan(self, msg):
    	readings = self._filterReadings(msg.ranges)
    	desiredAngle = self._sumComponentForces(zip(range(360), readings))


    def run(self):
        """ main behavior of wall follower class

        """
        rospy.init_node('obstacle_avoider', anonymous=True)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            self._velocityPublisher.publish(self._velocityMessage)
            r.sleep()

if __name__ == "__main__":
	obstacleAvoider = ObstacleAvoider()
	try:
		obstacleAvoider.run()
	except rospy.ROSInterruptException:pass