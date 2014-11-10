import numpy as np
import cv2
import copy
import random
import time
from CalibrationController import CalibrationController
from TrackingController import TrackingController
from NeatoController import NeatoController
import rospy

class HandGestureRecognizer(object):
	""" Class for converting video footage to a hand gesture 
	"""
	def __init__(self, videoFeed):
		""" 
		videoCapture: capture object returned from cv2.VideoCapture
		"""
		self._videoFeed = videoFeed
		self._calibrationController = CalibrationController(self._videoFeed)
		self._trackingController = TrackingController(self._videoFeed, [])
		self._neatoController = NeatoController()

	def calibrate(self):
		self._trackingController._calibrationColors = self._calibrationController.calibrateColors()
		print map(lambda x:list(x), self._trackingController._calibrationColors)

	def trackHand(self,calibrationColors=None):
		if calibrationColors != None:
			self._trackingController._calibrationColors = calibrationColors

		self._trackingController.on('up', self._neatoController.up)
		self._trackingController.on('left', self._neatoController.left)
		self._trackingController.on('right', self._neatoController.right)
		rospy.init_node('rospy_controller', anonymous=True)
		# r = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			self._trackingController._trackHand()

cap = cv2.VideoCapture(0)
h = HandGestureRecognizer(cap)
# h.calibrate()

colors = [[174, 175, 146], [178, 170, 141], [184, 176, 140], [183, 168, 132], [177, 171, 135], [182, 167, 131], [179, 183, 152], [197, 188, 155], [185, 187, 169], [191, 196, 182], [197, 196, 179], [194, 191, 167], [200, 195, 176], [189, 182, 150], [198, 186, 162], [196, 181, 153]]

h.trackHand(colors)