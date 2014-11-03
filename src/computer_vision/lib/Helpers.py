import cv2

class ImageProcessor(object):
	def __init__(self):
		pass

class ImageRenderer(object):
	def __init__(self):
		self.windows = {}

	def showWindows(self):
		for windowName, window in self.windows.items():
			for event in window.events:
				if event.shouldTrigger():
					event.handler()
			if window.shouldRender:
				cv2.imshow(windowName, window.image)

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
		if cv2.waitKey(20) & 0xFF == self.key:
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

