#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import sys, tty, termios
import threading
from StrokeWidthTransform.lib import textCropper
from StrokeWidthTransform.lib import translator
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

class NeatoTextLocalizer(object):
	def __init__(self, imgBuf=None, debugMode=False):
		rospy.init_node('NeatoTextLocalizer', anonymous=True)
		rospy.Subscriber("/camera/image_raw", Image, self.handle_video_update)
		self.armedToTranslate = False
		self.bridge = CvBridge()
		self.debugMode = debugMode
		self.stop = False
		self.imgBuf = imgBuf
		self.cropper = textCropper.TextCropper()
		self.translator = translator.Translator()
		self._lastChar = None


	@property 
	def lastChar(self):
		return self._lastChar

	@lastChar.setter
	def lastChar(self, newValue):
		self._lastChar = newValue

		if self._lastChar == 'q':
			print "Stopping Neato"
			self.stop = True
		elif self._lastChar == 't':
			print "Translating Image..."
			self.translateImage(self.imgBuf)

	def cropAndTranslateImage(self, img):
		regions = self.cropper.cropTextRegionsFromImage(img)
		print "%d Regions Detected" % len(regions)
		for i, region in enumerate(regions):
			b,g,r = cv2.split(region)
			rgbImg = cv2.merge([r,g,b])
			if self.debugMode:
				plt.subplot(len(regions), 1, i+1)
				plt.imshow(rgbImg)
			rospy.loginfo(self.translator.find_text(rgbImg))
		if self.debugMode:
			plt.show()


	def getch(self):
		""" Return the next character typed on the keyboard """
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		tty.setraw(sys.stdin.fileno())
		ch = sys.stdin.read(1)
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch

	def getKeyboardInput(self):
		while True:
			char = self.getch()
			self.lastChar = char
			if self.stop:
				break

	def handle_video_update(self, data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			self.imgBuf = image
		except CvBridgeError, e:
			print e

	def run(self):
		""" runs the neato node. press t to translate, press q to quit
		"""
		t = threading.Thread(target=self.getKeyboardInput)
		t.start()
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self.stop:
				break
			rate.sleep()

	def translateImage(self, img):
		t = threading.Thread(target=self.cropAndTranslateImage, args=(img,))
		t.start()


if __name__ == "__main__":
	print "Neato Text Localizer: "
	neato = NeatoTextLocalizer(debugMode=True)
	neato.run()