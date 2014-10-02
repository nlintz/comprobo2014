import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from random import random

if __name__ == "__main__":
	rospy.init_node('random_walk', anonymous=True)
	r = rospy.Rate(10)
	velocityPublisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
	while not rospy.is_shutdown():
		x = random()*1.0
		y = random()*1.0
		theta = random() * 3.14
		velocityPublisher.publish(Twist(linear=Vector3(x=x, y=y), angular=Vector3(z=theta)))
		r.sleep()