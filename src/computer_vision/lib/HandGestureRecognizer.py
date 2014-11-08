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
		print map(lambda x:list(x), self._trackingController._calibrationColors)

	def trackHand(self,calibrationColors=None):
		if calibrationColors != None:
			self._trackingController._calibrationColors = calibrationColors
		self._trackingController._trackHand()

cap = cv2.VideoCapture(0)
h = HandGestureRecognizer(cap)
# h.calibrate()

colors = [[206, 182, 174], [202, 188, 170], [199, 186, 185], [202, 170, 153], [208, 179, 166], [211, 166, 140], [200, 159, 141], [204, 166, 146], [202, 162, 142], [193, 161, 150], [199, 160, 142], [192, 170, 162], [200, 178, 170]]

h.trackHand(colors)