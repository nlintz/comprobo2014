import cv2
import numpy as np
import Helpers
import copy

class TrackingController(object):
	def __init__(self, videoFeed, calibrationColors=None):
		self.window = Helpers.Window("TrackingWindow")
		self.renderer = Helpers.ImageRenderer()
		self.renderer.addWindow(self.window)
		self._stopTracking = False
		self._videoFeed = videoFeed
		if calibrationColors == None:
			calibrationColors = []
		self._calibrationColors = calibrationColors

		trackingComplete = Helpers.KeyboardEvent()
		trackingComplete.registerKeyPressed('c', self._trackingComplete)

		self.window.registerEvent(trackingComplete)

	def _trackingComplete(self):
		self._stopTracking = True

	def _drawConvexHull(self, contours):
		convexHulls = TrackingController._contourConvexHulls(contours)
		for contour in contours:
			cv2.drawContours(self.window.image, [convexHulls[str(contour)]], -1, (0,0,255), 3)
			cv2.drawContours(self.window.image, [contour], -1, (0,255,0), 3)
			
	def _drawConvexityDefects(self, contours):
		convexityDefects = TrackingController._contourConvexityDefects(contours)
		for contour in contours:
			contourDefects = convexityDefects.get(str(contour), None)
			if contourDefects is not None:
				for i in range(contourDefects.shape[0]):
					s,e,f,d = contourDefects[i,0]
					defectPoint = tuple(contour[f][0])
					cv2.circle(self.window.image,defectPoint,5,[255,255,255],-1)

	@staticmethod
	def _contourConvexHulls(contours):
		contourConvexHulls = {}
		for contour in contours:
			hull = cv2.convexHull(contour)
			contourConvexHulls[str(contour)] = hull
		return contourConvexHulls

	@staticmethod
	def _contourConvexityDefects(contours):
		contourConvexityDefects = {}
		for contour in contours:
			hullIndices = cv2.convexHull(contour, returnPoints=False)
			if len(hullIndices)>3 and len(contour)>3:
				defects = cv2.convexityDefects(contour, hullIndices)
				contourConvexityDefects[str(contour)] = defects
				# pass
		return contourConvexityDefects


	@staticmethod
	def _contours(image, threshold=100):
		edges = cv2.Canny(copy.deepcopy(image), threshold, threshold*2)
		contours, _ = cv2.findContours(copy.deepcopy(edges), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		return contours	

	@staticmethod
	def _convexHullAndIndices(contour):
		return (cv2.convexHull(contour), cv2.convexHull(contour, returnPoints=False))

	@staticmethod
	def _filterHandFromImage(image, calibrationColors):
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		masks = [TrackingController._hsvMaskFromCalibrationColor(calibrationColors[i], hsv, 10) for i in range(len(calibrationColors))]
		maskSum = reduce(lambda x,y: cv2.add(x,y),masks)
		blurredMaskSum = cv2.medianBlur(maskSum, 9)
		return blurredMaskSum

	def _trackHand(self):
		while not self._stopTracking:
			ret, image = self._videoFeed.read()
			self.window.updateImage(np.zeros(image.shape, np.uint8))

			filteredImage = cv2.cvtColor(TrackingController._filterHandFromImage(image, self._calibrationColors), cv2.COLOR_GRAY2BGR)

			self.window.updateImage(filteredImage)

			contours = self._contours(filteredImage)
			self._drawConvexHull(contours)
			self._drawConvexityDefects(contours)
			self.renderer.showWindows()

	@staticmethod
	def _hsvMaskFromCalibrationColor(calibrationColor, hsvImage, sensitivity):
		maskColor = np.uint8([[ calibrationColor ]])
		hsvMaskColor = cv2.cvtColor(maskColor,cv2.COLOR_BGR2HSV)

		lowerBound = np.array([hsvMaskColor[0][0][0] - sensitivity, 50, 50])
		upperBound = np.array([hsvMaskColor[0][0][0] + sensitivity, 250, 250])
		mask = cv2.inRange(hsvImage, lowerBound, upperBound)
		return mask