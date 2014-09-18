#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

from PIDController import PIDController

class ObstacleAvoider(object):
    """ Class for handling wall following behavior

    """
    def __init__(self, targetAngle=0.0, pullForce=10.0, obstacleScaleInverseFactor=1.0):
        self._velocityPublisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self._scanSubscriber = rospy.Subscriber('scan', LaserScan, self._onScan)
        self._odomSubscriber = rospy.Subscriber('odom', Odometry, self._onOdom)
        self._velocityMessage = Twist()
        self._pullForce = pullForce
        self._obstacleScaleFactor = obstacleScaleInverseFactor
        self._currentAngle = None

        self._angleController = PIDController(targetAngle, 0.2, 0.001, 0.02)
        self._stop = False

    def stop():
        self._stop = True

    @staticmethod
    def _filterReadings(data, lowerBound=0.0, upperBound=10.0):
        return map(lambda x: x if x > lowerBound and x <= upperBound else float("inf"), data)

    @staticmethod
    def _odomToAngle(value, leftMin, leftMax, rightMin, rightMax):
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        valueScaled = float(value - leftMin) / float(leftSpan)
        return rightMin + (valueScaled * rightSpan)

    def _sumComponentForces(self, angleDistances):
        """ Sum the force components

        """
        pullAngle = self._odomToAngle(self._currentAngle, -1.0, 1.0, -3.14, 3.14)
        pullX, pullY = math.cos(pullAngle), math.sin(pullAngle)
        xComponents = pullX*self._pullForce
        yComponents = pullY*self._pullForce
        for angle, reading in angleDistances:
            if reading == float("inf"):
                continue
            angle = self._odomToAngle(angle, 0, 359, 0, 2*3.14) - pullAngle
            xComponents -= (self._obstacleScaleFactor / reading)*math.cos(angle)
            yComponents -= (self._obstacleScaleFactor / reading)*math.sin(angle)
        return xComponents, yComponents

    def _onOdom(self, msg):
        """ Odometry handler

        """
        self._currentAngle = msg.pose.pose.orientation.z

    def _onScan(self, msg):
        if self._currentAngle:
            readings = self._filterReadings(msg.ranges)
            desiredAngle = self._sumComponentForces(zip(range(360), readings))
            desiredAngle = -1.0 * math.atan2(desiredAngle[1],desiredAngle[0])
            self._angleController.target = desiredAngle
            parallelFeedback = -1.0 * self._angleController.getFeedback(self._currentAngle)
            print parallelFeedback
            self._velocityMessage = Twist(angular=Vector3(z=20*parallelFeedback), linear=Vector3(x=0.5))


    def run(self):
        """ main behavior of obstacle avoider class

        """
        self._stop = False
        rospy.init_node('obstacle_avoider', anonymous=True)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not self._stop:
                self._velocityPublisher.publish(self._velocityMessage)
                r.sleep()

if __name__ == "__main__":
	obstacleAvoider = ObstacleAvoider()
	try:
		obstacleAvoider.run()
	except rospy.ROSInterruptException:pass