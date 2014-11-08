import cv2
import time
from hashlib import sha1

from numpy import all, array, uint8

class ImageProcessor(object):
	def __init__(self):
		pass

class ImageRenderer(object):
	def __init__(self):
		self.windows = {}

	def showWindows(self):
		for windowName, window in self.windows.items():
			if window.shouldRender:
				cv2.imshow(windowName, window.image)
			if len(window.events):
				for event in window.events:
					if event.shouldTrigger():
						event.handler()
			else:
				if cv2.waitKey(10) & 0xFF == ord('q'):
					exit()

	def addWindow(self, window):
		cv2.namedWindow(window.name)
		self.windows[window.name] = window

class KeyboardEvent(object):
	def __init__(self):
		self.callback = None
		self.key = None

	def registerKeyPressed(self, key, callback=None, nonAlpha=False):
		""" registers a keypress event
		key: key which uses presses
		callback: function which should fire on keypress
		nonAscii: if nonAscii is false the user can pass in a key like 'a' and it will be converted to the ascii value 97
			if nonAscii is true, the user passes in the ascii value of the key they want to register
		"""
		if self.key == None:
			if nonAlpha:
				self.key = key
			else:
				self.key = ord(key)

		if self.callback == None and callback:
			self.callback = callback
		elif self.callback == None and callback == None:
			raise Exception("Must pass a callback to register keypress")
		if cv2.waitKey(10) & 0xFF == self.key:
			return True

	def shouldTrigger(self):
		if self.registerKeyPressed(self.key):
			return True

	def handler(self):
		self.callback()

class Window(object):
	def __init__(self, name, image=None):
		self.name = name
		self.image = image
		self.events = []
		self.shouldRender = True

	def registerEvent(self, event):
		self.events.append(event)

	def updateImage(self, image):
		self.image = image



class hashable(object):
	r'''Hashable wrapper for ndarray objects.

		Instances of ndarray are not hashable, meaning they cannot be added to
		sets, nor used as keys in dictionaries. This is by design - ndarray
		objects are mutable, and therefore cannot reliably implement the
		__hash__() method.

		The hashable class allows a way around this limitation. It implements
		the required methods for hashable objects in terms of an encapsulated
		ndarray object. This can be either a copied instance (which is safer)
		or the original object (which requires the user to be careful enough
		not to modify it).
	'''
	def __init__(self, wrapped, tight=False):
		r'''Creates a new hashable object encapsulating an ndarray.

			wrapped
				The wrapped ndarray.

			tight
				Optional. If True, a copy of the input ndaray is created.
				Defaults to False.
		'''
		self.__tight = tight
		self.__wrapped = array(wrapped) if tight else wrapped
		self.__hash = int(sha1(wrapped.view(uint8)).hexdigest(), 16)

	def __eq__(self, other):
		return all(self.__wrapped == other.__wrapped)

	def __hash__(self):
		return self.__hash

	def unwrap(self):
		r'''Returns the encapsulated ndarray.

			If the wrapper is "tight", a copy of the encapsulated ndarray is
			returned. Otherwise, the encapsulated ndarray itself is returned.
		'''
		if self.__tight:
			return array(self.__wrapped)

		return self.__wrapped

def subtractVectors(vectorA, vectorB):
	return (vectorA[0] - vectorB[0], vectorA[1] - vectorB[1])