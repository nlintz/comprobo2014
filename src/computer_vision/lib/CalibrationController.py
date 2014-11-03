import cv2
import numpy as np
import Helpers
import copy

class CalibrationController(object):
	def __init__(self, videoFeed):
		self.window = Helpers.Window("CalibrationWindow")
		self.renderer = Helpers.ImageRenderer()
		self.renderer.addWindow(self.window)
		
		self._calibrationPaused = False
		self._calibrationComplete = False
		self._videoFeed = videoFeed
		self._calibrationPoints = []
		self._calibrationImage = None

		# REGISTER CALIBRATION EVENTS
		unpause = Helpers.KeyboardEvent()
		unpause.registerKeyPressed(32, self._toggleCalibrationPaused, nonAlpha=True)
		calibrationComplete = Helpers.KeyboardEvent()
		calibrationComplete.registerKeyPressed('a', self._calibrationCompleted)
		cv2.setMouseCallback(self.window.name, self._setCalibrationPoint)
		
		self.window.registerEvent(unpause)
		self.window.registerEvent(calibrationComplete)

	def _calibrationCompleted(self):
		self._calibrationComplete = not self._calibrationComplete

	def _getCalibrationColors(self):
		calibrationColors = []
		for point in self._calibrationPoints:
			calibrationColors.append(self._calibrationImage[point[1], point[0]])
		return calibrationColors

	def _setCalibrationPoint(self, event, x, y, flag, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self._calibrationPoints.append((x, y))

	def _toggleCalibrationPaused(self):
		self._calibrationPaused = not self._calibrationPaused

	def _setCalibrationPoint(self, event, x, y, flag, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self._calibrationPoints.append((x, y))

	def calibrateColors(self):
		while not self._calibrationComplete:
			if not self._calibrationPaused:
				ret, img = self._videoFeed.read()
				self.window.updateImage(img)
				self._calibrationImage = copy.deepcopy(self.window.image)
			
			for point in self._calibrationPoints:
				cv2.circle(self.window.image, point, 5, (255,0,0), -1)
			self.renderer.showWindows()
		return self._getCalibrationColors()