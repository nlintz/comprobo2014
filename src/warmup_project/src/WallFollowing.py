#!/usr/bin/env python

import sys, os
import signal
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

from Teleop import teleop
from PIDController import PIDController

class WallFollower(object):
    """ Class for handling wall following behavior

    """
    def __init__(self, targetDistance=1, targetAngle=90):
        self.targetDistance = targetDistance
        self.targetAngle = targetAngle
        self._velocityPublisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self._scanSubscriber = rospy.Subscriber('scan', LaserScan, self.onScan)
        
        # self._wallFollowController = PIDController(self.targetDistance, 1.0, 0.00, 0.0001, tolerance=0.1)
        # self._distanceToWallController = PIDController(self.targetDistance, 1.0, 0.01, 0.05)
        self._angleController = PIDController(self.targetAngle, 0.02, 0.0001, 0.00)
        self._distanceController = PIDController(self.targetDistance, 0.5, 0.001, 0.00)
        self._velocityMessage = Twist()
        self._states = {"faceWall":self.faceWall, "moveToWall": self.moveToWall, "followWall":self.followWall, "turnLeft":self.turnLeft}
        self._currentState = "followWall" 
        # self._startAngle = None

    def stop(self):
        self._velocityMessage = Twist()
        self._velocityPublisher.publish(self._velocityMessage)

    @staticmethod
    def filterReadings(data, lowerBound=0.0, upperBound=10.0):
        return map(lambda x: x if x > lowerBound and x <= upperBound else float("inf") , data)

    @staticmethod
    def sampleReadings(data, startIndex, endIndex):
        """ returns the mean of a sample of readings

        startIndex -- Number starting index of sample
        endIndex -- Number end index of sample (end index is NOT included in result)
        """
        if (endIndex > len(data)):
            endIndex = len(data)
        validData = [data[i] for i in range(startIndex, endIndex) if data[i] != float("inf")]
        dataLength = endIndex - startIndex
        if len(validData) == dataLength:
            return sum(validData) / float(dataLength)
        else:
            return None

    def wallAngle(self, data):
        """ returns the angle of the wall relative to the robot in degrees

        """
        if (len(data)):
            return data.index(min(data))

    def onScan(self, msg):
        readings = self.filterReadings(msg.ranges)
        angle = self.wallAngle(readings)

        sample = self.sampleReadings(readings, angle, angle + 2)
        if sample:
            self._states[self._currentState](angle, sample)

    def turn(self, desiredAngle, currentAngle, nextState):
        if abs(desiredAngle - currentAngle) < 5:
            self._currentState = nextState
            self._velocityMessage = Twist()
        else:
            self._velocityMessage.angular.z = .5

    def followWall(self, angle, distance):
        parallelFeedback = self._angleController.getFeedback(angle)
        distanceFeedback = self._distanceController.getFeedback(distance)
        # angularVelocity = self._wallFollowController.getFeedback(reading)
        if angle <= 180:
            parallelFeedback *= -1
            distanceFeedback *= -1
        # self._velocityMessage.angular.z = parallelFeedback + distanceFeedback
        self._velocityMessage.angular.z = parallelFeedback + 2.25 * distanceFeedback
        # self._velocityMessage.angular.z = .1
        print "parallelFeedback: %f, distanceFeedback: %f" % (parallelFeedback, distanceFeedback)
        print "angle: %f, distance: %f" % (angle, distance)
        # self._velocityMessage.linear.x = -1.0 * distanceFeedback
        self._velocityMessage.linear.x = 0.35

    def moveToWall(self, angle, reading):
        if reading - self.targetDistance < .01:
            self._currentState = "turnLeft"
            self._velocityMessage = Twist()
        else:
            self._velocityMessage.linear.x = -1.0 * self._distanceToWallController.getFeedback(reading)

    def turnLeft(self, angle, reading):
        self.turn(90, angle, "followWall")

    def faceWall(self, angle, reading):
        self.turn(0, angle, "moveToWall")
            

    def run(self):
        """ main function of wall follower class

        """
        rospy.init_node('wall_follower', anonymous=True)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._velocityPublisher.publish(self._velocityMessage)
            r.sleep()


if __name__ == '__main__':
    wallFollower = WallFollower(1.0)
    
    try:
        wallFollower.run()
    except rospy.ROSInterruptException:pass