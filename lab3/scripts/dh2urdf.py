#! /usr/bin/python

import json
from tf.transformations import *
from collections import OrderedDict
import os

path = os.path.realpath(__file__)
with open(os.path.dirname(path) + '/../param_files/dh_file.json') as input_file:
    dh = json.loads(input_file.read(), object_pairs_hook=OrderedDict)

X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)

with open(os.path.dirname(path) + '/../param_files/urdf_params.yaml', 'w') as output_file:
    row_number = 0
    for row in dh:
        row_number += 1

        a, d, alpha, theta = dh[row]

        x_transl = translation_matrix((a, 0, 0))
        x_rot = rotation_matrix(alpha, X)
        z_transl = translation_matrix((0, 0, d))
        z_rot = rotation_matrix(theta, Z)

        transformation = concatenate_matrices(x_rot, x_transl, z_rot, z_transl)

        rpy = euler_from_matrix(transformation)
        xyz = translation_from_matrix(transformation)

        output_file.write("nr{}:".format(row_number) + "\n")
        output_file.write("  xyz: {} {} {}".format(*xyz) + "\n")
        output_file.write("  rpy: {} {} {}".format(*rpy) + "\n")
        output_file.write("  link_xyz: {} 0 0".format(xyz[0] / 2) + "\n")
        output_file.write("  link_rpy: 0 0 0\n")
        output_file.write("  link_length: {}".format(a) + "\n")
