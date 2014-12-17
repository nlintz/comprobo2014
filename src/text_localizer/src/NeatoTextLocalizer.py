#!/usr/bin/env python
import rospy
import cv2
import sys, tty, termios
import threading

class NeatoTextLocalizer(object):
	def __init__(self, imgBuf=None):
		rospy.init_node('NeatoTextLocalizer', anonymous=True)
		self.armedToTranslate = False
		self.stop = False
		self.imgBuf = imgBuf
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

	def getch(self):
	    """ Return the next character typed on the keyboard """
	    fd = sys.stdin.fileno()
	    old_settings = termios.tcgetattr(fd)
	    try:
	        tty.setraw(sys.stdin.fileno())
	        ch = sys.stdin.read(1)
	    finally:
	        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
	    return ch

	def getKeyboardInput(self):
		while True:
			char = self.getch()
			self.lastChar = char
			if self.stop:
				break

	def run(self):
		""" runs the neato node. press t to translate
		"""
		t = threading.Thread(target=self.getKeyboardInput)
		t.start()
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self.stop:
				break
			rate.sleep()

	def translateImage(self, img):
		print 'translate'


if __name__ == "__main__":
	neato = NeatoTextLocalizer()
	neato.run()