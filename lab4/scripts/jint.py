#! /usr/bin/python

import sys
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from lab4.srv import JintServiceStruct
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import os
import json
from PyKDL import *
from collections import OrderedDict


path = Path()
freq = 50

def getParams():
	file_path = os.path.realpath(__file__)
	with open(os.path.dirname(file_path) + '/../param_files/dh_file.json') as input_file:
		params = json.loads(input_file.read(), object_pairs_hook=OrderedDict)
	return params

def is_allowed_interpolation_request(req):
	for joint in [req.joint1, req.joint2, req.joint2]:
		if not -1.57075 <= joint <= 1.57075:
			rospy.logerr("Blad. Katy powinny byc w zakresie -1.57075 do 1.57075.")
			return False

	if req.time <= 0:
		rospy.logerr("Blad. Czas powinien byc wiekszy od 0.")
		return False

	if req.int_type != "quadratic" and req.int_type != "linear":
		rospy.logerr("Blad. Niepoprawny typ interpolacji. Poprawne typy: linear i quadratic.")
		return False

	return True


def linear_int(start_angle, end_angle, time, current_time):
	return start_angle + (float(end_angle - start_angle) / time) * current_time


def quadratic_int(start_angle, end_angle, time, current_time, a):
	if current_time < time / 2.:
		return start_angle + a * current_time**2
	else:
		return end_angle - a * (time - current_time)**2

def create_empty_joint_state():
	empty_joint_state = JointState()
	empty_joint_state.header = Header()
	empty_joint_state.header.stamp = rospy.Time.now()
	empty_joint_state.name = ['base_to_link_1', 'link1_to_link_2', 'link2_to_link_3']
	empty_joint_state.velocity = []
	empty_joint_state.effort = []
	return empty_joint_state

def compute_coefs(start_pos, request_pos, time): # wspolczynniki do interpolacji f kwadratowa
	a = []
	for i in range(0, len(start_pos)):
		a.append(2. * float(request_pos[i] - start_pos[i]) / time**2)
	return a

def publish_path(data):
	chain = Chain()
	angles = JntArray(3)
	kdl_frame = Frame()
	order = [2, 3, 1]

	for i in order:
		a, d, alpha, theta = params["i" + str(i)]

		joint = Joint(Joint.RotZ)
		frame = kdl_frame.DH_Craig1989(a, alpha, d, theta)
		chain.addSegment(Segment(joint, frame))

		angles[i-1] = data.position[i-1]

	fk_solver = ChainFkSolverPos_recursive(chain)
	result = Frame()
	fk_solver.JntToCart(angles, result)

	poseStamped = PoseStamped()
	poseStamped.header.stamp = data.header.stamp
	poseStamped.header.frame_id = "base_link"

	poseStamped.pose.position.x = result.p[0]
	poseStamped.pose.position.y = result.p[1]
	poseStamped.pose.position.z = result.p[2]

	xyzw = result.M.GetQuaternion()
	poseStamped.pose.orientation.x = xyzw[0]
	poseStamped.pose.orientation.y = xyzw[1]
	poseStamped.pose.orientation.z = xyzw[2]
	poseStamped.pose.orientation.w = xyzw[3]

	path.header = poseStamped.header
	path.poses.append(poseStamped)
	path_pub.publish(path)


def handle_interpolation_request(req):

	if not is_allowed_interpolation_request(req):
		return False

	start_pos = rospy.wait_for_message('joint_states', JointState, timeout = 10).position
	request_pos = [req.joint1, req.joint2, req.joint3]

	rate = rospy.Rate(freq)
	frames_number = int(math.ceil(req.time * freq))
	current_time = 0.

	if req.int_type == "linear":
		for k in range(0, frames_number + 1):
			joint_state = create_empty_joint_state()
			positions = []
			for i in range(0, 3):
				positions.append(linear_int(start_pos[i], request_pos[i], req.time, current_time))

			joint_state.position = positions
			pub.publish(joint_state)
			publish_path(joint_state)
			current_time = current_time + 1. / freq
			rate.sleep()
	else:
		a = compute_coefs(start_pos, request_pos, req.time) # wspolczynnki funkcji kwadratowych przy drugiej potedze
		for k in range(0, frames_number + 1):
			joint_state = create_empty_joint_state()
			positions = []
			for i in range(0, 3):
				positions.append(quadratic_int(start_pos[i], request_pos[i], req.time, current_time, a[i]))

			joint_state.position = positions
			pub.publish(joint_state)
			publish_path(joint_state)
			current_time = current_time + 1. / freq
			rate.sleep()

	return True


if __name__ == "__main__":
	params = getParams()
	rospy.init_node('jint')
	pub = rospy.Publisher('interpolation', JointState, queue_size = 10)
	path_pub = rospy.Publisher('jint_path', Path, queue_size=10)
	s = rospy.Service('jint_control_srv', JintServiceStruct, handle_interpolation_request)
	rospy.spin()