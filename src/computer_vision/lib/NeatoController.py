from geometry_msgs.msg import Twist, Vector3
import rospy

class NeatoController():
	def __init__(self):
		self.publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
	
	def left(self):
		self.publisher.publish(Twist(linear=Vector3(0.3, 0.0, 0), angular=Vector3(z=.8)))

	def right(self):
		self.publisher.publish(Twist(linear=Vector3(0.3, 0.0, 0), angular=Vector3(z=-.8)))

	def up(self):
		self.publisher.publish(Twist(linear=Vector3(0.5, 0.0, 0)))

	def stop(self):
		self.publisher.publish(Twist())

def main():
	commands = NeatoCommands()
	rospy.init_node('rospy_controller', anonymous=True)
	r = rospy.Rate(10) # 10hz
	# while not rospy.is_shutdown():
		