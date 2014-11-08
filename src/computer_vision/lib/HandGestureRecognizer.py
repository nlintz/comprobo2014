import numpy as np
import cv2
import copy
import random
import time
from CalibrationController import CalibrationController
from TrackingController import TrackingController

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

	def calibrate(self):
		self._trackingController._calibrationColors = self._calibrationController.calibrateColors()
		# print map(lambda x:list(x), self._trackingController._calibrationColors)

	def trackHand(self,calibrationColors=None):
		if calibrationColors != None:
			self._trackingController._calibrationColors = calibrationColors
		self._trackingController._trackHand()

cap = cv2.VideoCapture(0)
h = HandGestureRecognizer(cap)
# h.calibrate()
h.trackHand([[95, 107, 149], [102, 111, 149], [121, 126, 161], [116, 121, 164], [118, 126, 160], [119, 130, 162], [117, 126, 163], [109, 114, 136], [120, 122, 147], [111, 111, 141], [102, 104, 155]])