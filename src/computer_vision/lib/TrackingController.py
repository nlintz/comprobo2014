import cv2
import numpy as np
import Helpers
import copy
import time
import hashlib
from GestureMemory import GestureMemory, Gestures
import math

class TrackingController(object):
	def __init__(self, videoFeed, calibrationColors=None):
		self.window = Helpers.Window("TrackingWindow")
		self.filteredImageWindow = Helpers.Window("FilteredImage")
		self.filteredImageWindow.shouldRender = False	

		self.renderer = Helpers.ImageRenderer()
		self.renderer.addWindow(self.window)
		self.renderer.addWindow(self.filteredImageWindow)

		self._stopTracking = False
		self._videoFeed = videoFeed
		if calibrationColors == None:
			calibrationColors = []
		self._calibrationColors = calibrationColors

		trackingComplete = Helpers.KeyboardEvent()
		trackingComplete.registerKeyPressed('c', self._trackingComplete)
		self.window.registerEvent(trackingComplete)

		saveFilteredImage = Helpers.KeyboardEvent()
		saveFilteredImage.registerKeyPressed('s', self._saveFilteredImage)
		self.filteredImageWindow.registerEvent(saveFilteredImage)
		self.gestureMemory = GestureMemory()


	def _trackingComplete(self):
		self._stopTracking = True

	def _saveFilteredImage(self):
		cv2.imwrite(str(time.time())+'.png', self.filteredImageWindow.image)

	def _drawConvexHull(self, contours, convexHulls):
		for contour in contours:
			cv2.drawContours(self.window.image, [convexHulls[Helpers.hashable(contour)]], -1, (0,0,255), 3)
	
	def _drawContours(self, contours):
		for contour in contours:
			cv2.drawContours(self.window.image, [contour], -1, (0,255,0), 3)
			
	def _drawConvexityDefects(self, contours, convexityDefects):
		for contour in contours:
			contourDefects = convexityDefects.get(Helpers.hashable(contour), None)
			if contourDefects is not None:
				for i in range(contourDefects.shape[0]):
					s,e,f,d = contourDefects[i,0]
					defectPoint = tuple(contour[f][0])
					start = tuple(contour[s][0])
					end = tuple(contour[e][0])
					cv2.circle(self.window.image,defectPoint,5,[255,255,255],-1)
					cv2.line(self.window.image, end, defectPoint, (255,0,0), 2)

	def _drawFilteredImage(self, filteredImage):
		self.filteredImageWindow.updateImage(filteredImage)

	@staticmethod
	def _contourConvexHulls(contours):
		contourConvexHulls = {}
		for contour in contours:
			hull = cv2.convexHull(contour)
			contourConvexHulls[Helpers.hashable(contour)] = hull
		return contourConvexHulls

	@staticmethod
	def _contourConvexityDefects(contours):
		contourConvexityDefects = {}
		for contour in contours:
			hullIndices = cv2.convexHull(contour, returnPoints=False)
			if len(hullIndices)>3 and len(contour)>3:
				defects = cv2.convexityDefects(contour, hullIndices)
				contour.flags.writeable = False
				contourConvexityDefects[Helpers.hashable(contour)] = defects
				# pass
		return contourConvexityDefects

	@staticmethod
	def _largestContours(contours, convexityDefects):
		largestContours = []

		for contour in contours:
			minX, minY, maxX, maxY = None, None, None, None

			contourDefects = convexityDefects.get(Helpers.hashable(contour), None)
			if contourDefects is not None:
				for i in range(contourDefects.shape[0]):
					s,e,f,d = contourDefects[i,0]
					defectPoint = tuple(contour[f][0])
					if minX is None:
						minX, maxX = defectPoint[0], defectPoint[0]
						minY, maxY = defectPoint[1], defectPoint[1]
						largestContour = contour
					else:
						if defectPoint[0] < minX:
							minX = defectPoint[0]
						if defectPoint[0] > maxX:
							maxX = defectPoint[0]
						if defectPoint[1] < minY:
							minY = defectPoint[1]
						if defectPoint[1] > maxY:
							maxY = defectPoint[0]
				largestContours.append((abs(maxX - minX) * abs(maxY - minY), contour))
		
		return sorted(largestContours, key=lambda x:x[0], reverse=True)

	def _evaluateFingerGesture(self, contours, convexityDefects):
		depthsAndPoints = []
		for contour in contours:
			contourDefects = convexityDefects.get(Helpers.hashable(contour), None)
			if contourDefects is not None:
				for i in range(contourDefects.shape[0]):
					start,end,depth_point,depth = contourDefects[i,0]
					depthsAndPoints.append((depth, tuple(contour[depth_point][0])))
		
		if len(depthsAndPoints):
			depths = map(lambda x:x[0], depthsAndPoints)
			depthPoints = map(lambda x:x[1], depthsAndPoints)
			maxDepth = max(depths)

			centerOfMass = (int(sum(map(lambda x: x[0], depthPoints)) / float(len(depthPoints))), 
				int(sum(map(lambda x: x[1], depthPoints)) / float(len(depthPoints))))
			fingerPoint = depthPoints[depths.index(maxDepth)]

			if len(filter(lambda x: x>maxDepth/2, depths)) < 5:
				"this happened"
				self.gestureMemory.receiveGesture(Gestures.Go, centerOfMass)
				return (Gestures.Go, centerOfMass, fingerPoint)

			else:
				return (Gestures.Stop)
		return Gestures.No_Gesture

	@staticmethod
	def _fingerDirection(gesture):
		gestureType, centerOfMass, fingerPoint = gesture
		x, y = Helpers.subtractVectors(fingerPoint, centerOfMass)
		fingerAngle = math.degrees(math.atan2(y, x))
		fingerAngleDirections = [("RIGHT",0), ("UP",90), ("LEFT",180)]
		direction = (np.abs(np.array(map(lambda x:x[1], fingerAngleDirections)) - abs(fingerAngle))).argmin()
		return map(lambda x:x[0], fingerAngleDirections)[direction]


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

	def _trackHand(self, shouldRender=True):
		while not self._stopTracking:
			ret, image = self._videoFeed.read()
			self.window.updateImage(np.zeros(image.shape, np.uint8))
			self.filteredImageWindow.updateImage(np.zeros(image.shape, np.uint8))

			filteredImage = cv2.cvtColor(TrackingController._filterHandFromImage(image, self._calibrationColors), cv2.COLOR_GRAY2BGR)
			contours = self._contours(filteredImage)
			convexityDefects = TrackingController._contourConvexityDefects(contours)
			convexHulls = TrackingController._contourConvexHulls(contours)
			contourSizes = self._largestContours(contours, convexityDefects)
			largestContours = [countourMass[1] for countourMass	in contourSizes]
			contours = largestContours[0:2]
			
			gesture = self._evaluateFingerGesture(contours, convexityDefects)
			if gesture == None:
				print "this should never happen"
				continue

			if gesture == Gestures.Stop:

				print "STOP"
			elif gesture == Gestures.Go:
				print "got hereeeee"
				gestureType, centerOfMass, fingerPoint = gesture
				cv2.circle(self.window.image,centerOfMass,10,[0,100,255],-1)
				cv2.circle(self.window.image,fingerPoint,20,[255,100,255],-1)
				# x, y = Helpers.subtractVectors(fingerPoint, centerOfMass)
				# fingerAngle = math.degrees(math.atan2(y, x))
				# fingerAngleDirections = [("RIGHT",0), ("UP",90), ("LEFT",180)]
				# direction = (np.abs(np.array(map(lambda x:x[1], fingerAngleDirections)) - abs(fingerAngle))).argmin()
				# print map(lambda x:x[0], fingerAngleDirections)[direction]
				print TrackingController._fingerDirection(gesture)


			if shouldRender:
				self._drawFilteredImage(filteredImage)
				# self._drawContours(contours)
				self._drawConvexityDefects(contours, convexityDefects)
				self._drawConvexHull(contours, convexHulls)
				self.renderer.showWindows()

	@staticmethod
	def _hsvMaskFromCalibrationColor(calibrationColor, hsvImage, sensitivity):
		maskColor = np.uint8([[ calibrationColor ]])
		hsvMaskColor = cv2.cvtColor(maskColor,cv2.COLOR_BGR2HSV)

		lowerBound = np.array([hsvMaskColor[0][0][0] - sensitivity, 50, 50])
		upperBound = np.array([hsvMaskColor[0][0][0] + sensitivity, 250, 250])
		mask = cv2.inRange(hsvImage, lowerBound, upperBound)
		return mask

# colors = [[220, 160,  92], [215, 152,  59], [199, 148,  79], [184, 143,  85], [188, 136,  56], [208, 158,  93], [204, 152,  51], [153, 126,  66], [188, 135,  51], [183, 137,  67], [169, 127,  65], [166, 128,  74], [185, 136,  68], [177, 141,  95], [192, 146,  82]]
# cap = cv2.VideoCapture(0)
# T = TrackingController(cap, colors)
# while 1:
# 	T._trackHand()

#ONE FINGER = [14306, 14306, 2737, 2737, 2676, 2676, 2412, 2329, 1262, 1262, 772, 772, 243, 243, 227, 227, 222, 222, 210, 210, 190, 190, 186, 186, 186, 186, 186, 186, 176, 176, 114, 114, 114, 114, 114, 114, 114, 114, 114, 114]
#TWO FINGERS = [27656, 27656, 12466, 12448, 5532, 5532, 3845, 3788, 1932, 1932, 878, 878, 805, 805, 744, 744, 519, 519, 256, 256, 186, 162, 162, 162, 162, 114, 114, 114, 114, 114, 114, 114, 114]
#THREE FINGERS = [22703, 22694, 21990, 21990, 5939, 5939, 4312, 4311, 2513, 2500, 931, 869, 162, 114, 114, 114, 114, 114, 114, 114, 114]
#FOUR FINGERS = [28157, 28157, 26261, 26173, 23948, 23948, 7161, 7161, 5445, 5445, 1586, 1577, 226, 226, 217, 217, 162, 162, 162, 162, 142, 142, 114, 114, 114, 114, 114, 114, 114, 114]
#FIVE FINGERS = [25205, 25205, 24124, 24033, 21163, 21163, 20783, 20767, 7027, 6988, 5993, 5993, 1445, 1439, 543, 512, 405, 162, 162, 142, 114, 114, 114, 114, 114, 114, 114, 114]
