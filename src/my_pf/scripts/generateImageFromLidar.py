#!/usr/bin/env python

import math
import numpy
import Image
import rospy
from sensor_msgs.msg import LaserScan

def _translate(value, leftMin, leftMax, rightMin, rightMax):
	leftSpan = leftMax - leftMin
	rightSpan = rightMax - rightMin
	valueScaled = float(value - leftMin) / float(leftSpan)
	return rightMin + (valueScaled * rightSpan)


def distanceToImage(resolution, distances, pointSize=3):
	minDistance, maxDistance = min(distances), max(distances)
	cartesian = numpy.zeros((resolution)+(3,), dtype = numpy.uint8)
	center = (resolution[0] / 2, resolution[1] / 2)
	centerX, centerY = 0.0, 0.0
	heading = 0.0

	for i, distance in enumerate(distances):
		particle_offset_angle = 30.0
		particle_offset_x = 0.0
		particle_offset_y = 0.0


		degrees_theta = i + 90.0 + particle_offset_angle
		angle = _translate(degrees_theta, 0.0, 360.0, 0.0, 2*math.pi)
		x = _translate(particle_offset_x + math.cos(angle) * distance, 0, maxDistance, 0, center[0] - 1)
		y = _translate(particle_offset_y + math.sin(angle) * distance, 0, maxDistance, 0, center[1] - 1)

		for j in range(int(-1.0 * pointSize / 2), int(pointSize / 2 + 1)):
			for k in range(int(-1.0 * pointSize / 2), int(pointSize / 2 + 1)):
				pointX = center[1] - int(y) + j
				pointY = int(x) + center[0] + k
				if pointX > 0 and pointX < resolution[0] and pointY > 0 and pointY < resolution[1]:
					cartesian[pointX][pointY][:] = [255, 255, 255]
	img = Image.fromarray(cartesian)
	img.save('myimg.jpeg')

def scan_received(msg):
	distanceToImage((500,500), msg.ranges)

if __name__ == '__main__':
	rospy.init_node('image_gen')
	r = rospy.Rate(5)
	laser_subscriber = rospy.Subscriber("scan", LaserScan, scan_received, queue_size=10)
	while not(rospy.is_shutdown()):
		r.sleep()
