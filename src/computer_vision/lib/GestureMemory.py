import numpy as np
import cv2
import copy
import random
import time
from enum import Enum


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
		self._currentGesture = Gesture.No_Gesture
		self._position

	def receiveGesture(self, scores, defects):
		current_time = time.time()
		self._timeBetweenGestures = current_time - self._timeOfLastGesture
		self._timeOfLastGesture = current_time
		if self._timeBetweenGestures > 0.5:
			self._gestureBuffer = 0
		
		x,y = 0,0
		for defect in defects:
			x+=defect[0]
			y+=defect[1]
		position = [float(x)/len(defects), float(y)/len(defects)]
		self._positionOfLastGesture = position

		probabilities = [float(i)/sum(scores) for i in scores]
		self._gestureBuffer.append(probabilities, position)

	def updateGesture(self):
		if len(self._gestureBuffer > 5):
			self._gestureBuffer = self._gestureBuffer[:len(self._gestureBuffer) - 5]

		gestures = [0.2,0.2,0.2,0.2,0.2]
		for gesture in self._gestureBuffer:
			for i in range(5):
				gestures[i]*= gesture[i]

		normalized = [float(i)/sum(gestures) for i in gestures]
		self._currentGesture = Gesture(normalized.index(max(normalized)))




class Gesture(Enum):
	Stop = 1
	Go = 2
	Left = 3
	Right = 4
	No_Gesture = 5







