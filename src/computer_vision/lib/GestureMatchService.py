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

# TODO create a training set + test