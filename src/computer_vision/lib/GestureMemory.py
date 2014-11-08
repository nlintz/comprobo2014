import numpy as np
import cv2
import copy
import random
import time
from enum import IntEnum


class GestureMemory(object):
	""" Class keeping track of gesture information for detecting moving gestures and
	improving accuracy of still gesture recognition 
	"""
	def __init__(self):
		import time
		self._timeOfLastGesture = 0 #ms
		self._positionOfLastGesture = [0,0]
		self._timeBetweenGestures = 0
		self._gestureBuffer = []
		self._currentGesture = Gestures.No_Gesture

	def receiveGesture(self, gesture, centerOfMass):
		current_time = time.time()
		self._timeBetweenGestures = current_time - self._timeOfLastGesture
		self._timeOfLastGesture = current_time
		if self._timeBetweenGestures > 0.5:
			self._gestureBuffer = []

		scores = [0.1,0.1,0.1,0.1,0.1]
		scores[gesture.value] = 0.6
		
		self._positionOfLastGesture = centerOfMass

		self._gestureBuffer.append([scores, centerOfMass])

	def updateGesture(self):
		if len(self._gestureBuffer > 5):
			self._gestureBuffer = self._gestureBuffer[:len(self._gestureBuffer) - 5]

		gestures = [0.2,0.2,0.2,0.2,0.2]
		for gesture in self._gestureBuffer:
			for i in range(5):
				gestures[i]*= gesture[0][i]

		normalized = [float(i)/sum(gestures) for i in gestures]
		self._currentGesture = Gesture(normalized.index(max(normalized)))




class Gestures(IntEnum):
	Stop = 1
	Go = 2
	Left = 3
	Right = 4
	No_Gesture = 5







