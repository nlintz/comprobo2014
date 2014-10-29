import numpy as np
import cv2
import copy

class HandGestureRecognizer(object):
	""" Class for converting video footage to a hand gesture 
	"""
	def __init__(self, videoFeed):
		""" 
		videoCapture: capture object returned from cv2.VideoCapture
		"""
		self._videoFeed = videoFeed
		self._calibrationPoints = []

	def calibrate(self):
		""" creates a calibration window. User clicks points on their hand 
		"""
		cv2.namedWindow("Calibration_Window")
		cv2.setMouseCallback("Calibration_Window", self._setCalibrationPoint)
		videoPaused = False

		while True:
			if cv2.waitKey(20) & 0xFF == 32:
				videoPaused = not videoPaused
			if cv2.waitKey(20) & 0xFF == 97:
				break
			if videoPaused == False:
				ret, img = self._videoFeed.read()
				calibrationImage = copy.deepcopy(img)
			for point in self._calibrationPoints:
				cv2.circle(calibrationImage, point, 5, (255,0,0), -1)
			cv2.imshow("Calibration_Window", calibrationImage)
		self._calibrationComplete(img)

	def _calibrationComplete(self, calibrationImage):
		calibrationColors = self._getCalibrationColors(calibrationImage)

		# TODO - THIS METHOD SHOULDNT BE ON THE GESTURE RECOGNIZER
		self._trackHand(calibrationColors)

	def _getCalibrationColors(self, calibrationImage):
		calibrationColors = []
		for point in self._calibrationPoints:
			calibrationColors.append(calibrationImage[point[1], point[0]])
		return calibrationColors

	def _setCalibrationPoint(self, event, x, y, flag, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self._calibrationPoints.append((x, y))

	def _trackHand(self, calibrationColors):
		cv2.namedWindow("Gesture_Tracking")

		while True:
			if cv2.waitKey(20) & 0xFF == 99:
				break
			_, img = self._videoFeed.read()
			hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
			maskColor = np.uint8([[ calibrationColors[0] ]])
			hsvMaskColor = cv2.cvtColor(maskColor,cv2.COLOR_BGR2HSV)

			lowerBound = np.array([hsvMaskColor[0][0][0] - 10, 50, 50])
			upperBound = np.array([hsvMaskColor[0][0][0] + 10, 250, 250])
			print lowerBound, upperBound

			mask = cv2.inRange(hsv, lowerBound, upperBound)

			cv2.imshow("Gesture_Tracking", mask)
