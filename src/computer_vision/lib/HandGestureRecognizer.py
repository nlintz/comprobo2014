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
		# self._trackingController = TrackingController(self._videoFeed, [])

	def calibrate(self):
		self._trackingController._calibrationColors = self._calibrationController.calibrateColors()

	def trackHand(self,calibrationColors=None):
		if calibrationColors != None:
			self._trackingController._calibrationColors = calibrationColors
		self._trackingController._trackHand()

cap = cv2.VideoCapture(0)
h = HandGestureRecognizer(cap)
h.calibrate()
# h.trackHand()