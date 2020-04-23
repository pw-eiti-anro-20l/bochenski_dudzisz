#! /usr/bin/python

import json
from PyKDL import *
from collections import OrderedDict
import os
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

def checkRestrictions(data, rest):
    for i in range(1,4):
        if data.position[i-1] < rest["i" + str(i)][0] or data.position[i-1] > rest["i" + str(i)][1]:
            return False
    return True

def getParams():
    path = os.path.realpath(__file__)
    with open(os.path.dirname(path) + '/../param_files/dh_file.json') as input_file:
        params = json.loads(input_file.read(), object_pairs_hook=OrderedDict)
    return params

def callback(data):
    if not checkRestrictions(data, rest):
        rospy.logerr('Position not possible')
        return

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

    pub = rospy.Publisher('kdl_dkin_msgs', PoseStamped, queue_size=10)
    pub.publish(poseStamped)

def kdl_dkin():
    rospy.init_node('KDL_DKIN', anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    params = getParams()

    rest = {}
    path = os.path.realpath(__file__)
    with open(os.path.dirname(path) + '/../restrictions.json') as input_file:
        rest = json.loads(input_file.read(), object_pairs_hook=OrderedDict) 
    try:
        kdl_dkin()
    except rospy.ROSInterruptException:
        pass
