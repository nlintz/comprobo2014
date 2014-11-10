import cv2
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))
from Helpers import *
import numpy as np

def returnTrue():
	print 'a pressed'
	return True

fixture_keyEvent_pressedA = KeyboardEvent()
fixture_keyEvent_pressedA.registerKeyPressed('a', returnTrue)

fixture_window = Window('GestureWindow', np.zeros((100, 100, 3)))
fixture_window.registerEvent(fixture_keyEvent_pressedA)

def test_ImageRenderer():
	renderer = ImageRenderer()
	renderer.addWindow(fixture_window)
	while 1:
		renderer.showWindows()

test_ImageRenderer()