#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))
import thread

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

from Teleop import teleop, getch
from PIDController import PIDController

char = None

class WallFollower(object):
    """ Class for handling wall following behavior

    """
    def __init__(self, targetDistance=1, targetAngle=90):
        self.targetDistance = targetDistance
        self.targetAngle = targetAngle
        self._velocityPublisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        # self._velocityPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._scanSubscriber = rospy.Subscriber('scan', LaserScan, self._onScan)
        
        self._angleController = PIDController(self.targetAngle, 0.02, 0.0001, 0.00)
        self._distanceController = PIDController(self.targetDistance, 0.5, 0.001, 0.00)
        self._velocityMessage = Twist()
        self._states = {"followWall":self._followWall, "teleop":self._teleop}
        self._currentState = "followWall"
        self._keyDirection = None
        self._stop = False

    def stop():
        self._stop = True

    @property 
    def state(self):
        return self._currentState

    @state.setter
    def state(self, state):
        self._currentState = state

    @property 
    def key(self):
        return self._keyDirection

    @key.setter
    def key(self, key):
        self._keyDirection = key

    @staticmethod
    def _filterReadings(data, lowerBound=0.0, upperBound=10.0):
        return map(lambda x: x if x > lowerBound and x <= upperBound else float("inf") , data)

    @staticmethod
    def _sampleReadings(data, startIndex, endIndex):
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

    @staticmethod
    def _wallAngle(data):
        """ returns the angle of the wall relative to the robot in degrees

        """
        if (len(data)):
            return data.index(min(data))

    def _onScan(self, msg):
        """ callback handler for scan event

        """
        readings = self._filterReadings(msg.ranges)
        angle = self._wallAngle(readings)

        distance = self._sampleReadings(readings, angle, angle + 5)
        distance = readings[angle]
        if distance:
            self._states[self._currentState](angle, distance)

    def _followWall(self, angle, distance):
        """ wall following behavior. Uses two complementary PID controllers to 
            keep the robot along the wall

        """
        parallelFeedback = self._angleController.getFeedback(angle)
        distanceFeedback = self._distanceController.getFeedback(distance)
        if angle <= 180:
            parallelFeedback *= -1
            distanceFeedback *= -1
        self._velocityMessage.angular.z = parallelFeedback + 2.25 * distanceFeedback
        self._velocityMessage.linear.x = 0.35

    def _teleop(self, angle, distance):
        char = self.key
        if char == 'w':
            self._velocityMessage = Twist(linear=Vector3(x=0.5))
        elif char == 's':
            self._velocityMessage = Twist(linear=Vector3(x=-0.5))
        if char == 'a':
            self._velocityMessage = Twist(angular=Vector3(z=0.5))
        elif char == 'd':
            self._velocityMessage = Twist(angular=Vector3(z=-0.5))

    def run(self):
        """ main behavior of wall follower class

        """
        self._stop = False
        rospy.init_node('wall_follower', anonymous=True)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not self._stop:
                self._velocityPublisher.publish(self._velocityMessage)
                r.sleep()

if __name__ == '__main__':
    wallFollower = WallFollower(.6)
    print ("Press `f` to follow walls\n"
            "Press `t` to control with keyboard\n"
            "Press `x` to exit")

    def keypress():
        while True:
            char = getch()
            if char == "x":
                rospy.signal_shutdown("exit")
                exit()
            elif char == "f":
                wallFollower._stop()
                wallFollower.state = 'followWall'
            elif char == "t":
                wallFollower._stop()
                wallFollower.state = 'teleop'
            else:
                wallFollower.key = char

    thread.start_new_thread(keypress, ())
    try:
        wallFollower.run()
    except rospy.ROSInterruptException:pass