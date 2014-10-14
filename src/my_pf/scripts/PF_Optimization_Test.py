#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist 

MODEL_NAME = "mobile_base"
last_pose = None
least_squares = 0

def callback(data):
    pose_index = data.name.index(MODEL_NAME)
    robot_pose = data.pose[pose_index]
    global last_pose
    last_pose = robot_pose

def pose_callback(data):
	# print "actual pose:", last_pose 
	# print "estimated pose:", data
	global least_squares
	least_squares += ((last_pose.position.x - data.position.x)**2 + (last_pose.position.y - data.position.y)**2)**.5
	print least_squares
    
def listener():
    rospy.init_node('optimization_test')
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    rospy.Subscriber("/pose_estimate", Pose, pose_callback)
    rospy.spin()

if __name__ == "__main__":
	listener()