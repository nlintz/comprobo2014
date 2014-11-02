import numpy as np
import cv2
import copy
import random

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
				print 'calibratoin complete'
				break
			if videoPaused == False:
				ret, img = self._videoFeed.read()
				calibrationImage = copy.deepcopy(img)
			for point in self._calibrationPoints:
				cv2.circle(calibrationImage, point, 5, (255,0,0), -1)
			cv2.imshow("Calibration_Window", calibrationImage)
		# TODO - decouple from calibration code
		self._calibrationComplete(img)

	def _calibrationComplete(self, calibrationImage):
		calibrationColors = self._getCalibrationColors(calibrationImage)

		# TODO - THIS METHOD SHOULDNT BE ON THE GESTURE RECOGNIZER
		self._trackHand(calibrationColors)

	def _convexHull(self, img, threshold=100):
		"""
		Returns convex hull for a black and white image
		"""
		# TODO, this method should be returning the convex hull and defects points instead of a plot with both
		edges = cv2.Canny(copy.deepcopy(img), threshold, threshold*2)
		convexHullImage = np.zeros(img.shape+(3,), np.uint8)

		contours, _ = cv2.findContours(copy.deepcopy(edges), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for cnt in contours:
			hull = cv2.convexHull(cnt)
			hullIndices = cv2.convexHull(cnt, returnPoints=False)
			cv2.drawContours(convexHullImage,[hull],-1,(0,0,255),3)
			cv2.drawContours(convexHullImage,[cnt],-1,(0,255,0),3)
			if len(hull)>3 and len(cnt)>3:
				defects = cv2.convexityDefects(cnt,hullIndices)
				if defects != None:
					for i in range(defects.shape[0]):
						s,e,f,d = defects[i,0]
						start = tuple(cnt[s][0])
						end = tuple(cnt[e][0])
						far = tuple(cnt[f][0])
						cv2.circle(convexHullImage,far,5,[255,255,255],-1)

		return convexHullImage

	def _getCalibrationColors(self, calibrationImage):
		calibrationColors = []
		for point in self._calibrationPoints:
			calibrationColors.append(calibrationImage[point[1], point[0]])
		return calibrationColors

	def _setCalibrationPoint(self, event, x, y, flag, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self._calibrationPoints.append((x, y))

	def _detectHand(self, img, calibrationColors):
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		masks = [HandGestureRecognizer._hsvMaskFromCalibrationColor(calibrationColors[i], hsv, 10) for i in range(len(calibrationColors))]
		maskSum = reduce(lambda x,y: cv2.add(x,y) ,masks)
		blurredMaskSum = cv2.medianBlur(maskSum, 9)
		return blurredMaskSum

	def _trackHand(self, calibrationColors):
		cv2.namedWindow("Gesture_Tracking")
		while True:
			if cv2.waitKey(20) & 0xFF == 99:
				break
			_, img = self._videoFeed.read()
			blurredMaskSum = self._detectHand(img, calibrationColors)
			convexHull = self._convexHull(blurredMaskSum, 100)
			cv2.imshow("Gesture_Tracking", convexHull)

	@staticmethod
	def _hsvMaskFromCalibrationColor(calibrationColor, hsvImage, sensitivity):
		maskColor = np.uint8([[ calibrationColor ]])
		hsvMaskColor = cv2.cvtColor(maskColor,cv2.COLOR_BGR2HSV)

		lowerBound = np.array([hsvMaskColor[0][0][0] - sensitivity, 50, 50])
		upperBound = np.array([hsvMaskColor[0][0][0] + sensitivity, 250, 250])
		mask = cv2.inRange(hsvImage, lowerBound, upperBound)
		return mask
