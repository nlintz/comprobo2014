import cv2
import numpy as np
import Helpers
import copy

class Gesture(object):
	def __init__(self, gestureName, trainingImages, contourExtractionFunction):
		""" Gesture object represents a hand gesture and the associated training data
		gestureName: string of the gesture type
		trainingImages: black and white images of a training gesture
		contourExtractionFunction: function to extract a contour from a black and white image
		"""
		self._gestureName = gestureName
		self._trainingContours = map(lambda x:contourExtractionFunction(x), trainingImages)

	@property 
	def name(self):
		return self._gestureName

	def similarity(self, contour):
		similarity = 1.0
		for contourSample in self._trainingContours:
			similarity *= 1.0/(.000001 + cv2.matchShapes(contourSample, contour))
		return similarity

def contourSimilarity():
	FFA = cv2.imread('hand_gestures/five_finger/1415122899.02.png',0)
	# FFB = cv2.imread('hand_gestures/five_finger/1415122924.83.png',0)
	FFB = cv2.imread('hand_gestures/two_finger/1415123092.48.png',0)
	sift = cv2.SIFT()
	kp1, des1 = sift.detectAndCompute(FFA,None)
	kp2, des2 = sift.detectAndCompute(FFB,None)
	FLANN_INDEX_KDTREE = 0
	index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
	search_params = dict(checks = 50)

	flann = cv2.FlannBasedMatcher(index_params, search_params)

	matches = flann.knnMatch(des1,des2,k=2)

	# store all the good matches as per Lowe's ratio test.
	good = []
	for m,n in matches:
		if m.distance < 0.7*n.distance:
			good.append(m)
	print sum(sorted([g.distance for g in good]))/float(len(good))

contourSimilarity()