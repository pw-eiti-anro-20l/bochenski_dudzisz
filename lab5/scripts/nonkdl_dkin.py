#! /usr/bin/python

import json
from tf.transformations import *
from collections import OrderedDict
import os
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

def getParams():
    path = os.path.realpath(__file__)
    with open(os.path.dirname(path) + '/../param_files/dh_file.json') as input_file:
        params = json.loads(input_file.read(), object_pairs_hook=OrderedDict)
    return params

def callback(data):
    X, Z = (1, 0, 0), (0, 0, 1)
    result = translation_matrix((0,0,0))
 
    for i in range(1,4):
        a, d, alpha, theta = params["i" + str(i)]
        
        x_transl = translation_matrix((a, 0, 0))
        x_rot = rotation_matrix(alpha, X)
        z_transl = translation_matrix((0, 0, d))
        z_rot = rotation_matrix(data.position[i-1], Z)

        transformation = concatenate_matrices(x_rot, x_transl, z_rot, z_transl)
        result = concatenate_matrices(result, transformation)
    
    #tool
    x_transl = translation_matrix((0.2, 0, 0))
    x_rot = rotation_matrix(0, X)
    z_transl = translation_matrix((0, 0, 0))
    z_rot = rotation_matrix(0, Z)

    transformation = concatenate_matrices(x_rot, x_transl, z_rot, z_transl)
    result = concatenate_matrices(result, transformation)

    poseStamped = PoseStamped()
    poseStamped.header.stamp = data.header.stamp
    poseStamped.header.frame_id = "base_link"

    xyz = translation_from_matrix(result)
    poseStamped.pose.position.x = xyz[0]
    poseStamped.pose.position.y = xyz[1]
    poseStamped.pose.position.z = xyz[2]
    
    xyzw = quaternion_from_matrix(result)
    poseStamped.pose.orientation.x = xyzw[0]
    poseStamped.pose.orientation.y = xyzw[1]
    poseStamped.pose.orientation.z = xyzw[2]
    poseStamped.pose.orientation.w = xyzw[3]

    pub = rospy.Publisher('nonkdl_dkin_msgs', PoseStamped, queue_size=10)
    pub.publish(poseStamped)

def nonkdl_dkin():
    rospy.init_node('NONKDL_DKIN', anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    params = getParams()    
    try:
        nonkdl_dkin()
    except rospy.ROSInterruptException:
        pass
