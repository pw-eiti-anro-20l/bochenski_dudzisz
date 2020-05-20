#!/usr/bin/env python

from math import cos, sin, sqrt, tan
import sys
import rospy
from lab5.srv import OintServiceStruct
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class Point(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


start_p = Point(0.3771, 0, 0.1089)
path = Path()


def move_within_start_points(how):
    if how != 'forward' and how != 'backward':
        rospy.logerr('Invalid move_within_start_points function \'how\' arg.')

    interpolation = rospy.ServiceProxy('/oint_control_srv', OintServiceStruct)
    if how == 'forward':
        interpolation(0.67, 0, 0.2, 0, 0, 0, 1, 0.3, 'linear')
        interpolation(0.5, 0, 0.2, 0, 0, 0, 1, 0.3, 'linear')
        interpolation(start_p.x, start_p.y, start_p.z, 0, 0, 0, 1, 0.3, 'linear')
    else:
        interpolation(0.5, 0, 0.2, 0, 0, 0, 1, 0.3, 'linear')
        interpolation(0.67, 0, 0.2, 0, 0, 0, 1, 0.3, 'linear')
        interpolation(0.7, 0, 0, 0, 0, 0, 1, 0.3, 'linear')



def draw_rectangle(times):
    x = start_p.x
    y = 0.30 # max - 0.58, musi pociagac za soba zmiane z_top
    z_bottom = start_p.z
    z_top = 0.4
    
    points = [Point(x, -y, z_bottom), Point(x, -y, z_top), Point(x, y, z_top), Point(x, y, z_bottom), Point(start_p.x, start_p.y, start_p.z)]

    # przemieszczenie do punktu startowego:
    move_within_start_points('forward')
    interpolation = rospy.ServiceProxy('/oint_control_srv', OintServiceStruct)
    for i in range(0, 5 * times):
        interpolation(points[i % 5].x, points[i % 5].y, points[i % 5].z, 0, 0, 0, 1, 0.5, 'linear')


    move_within_start_points('backward')


def draw_elipse(times):
    # ruch bedzie odbywal sie z punktu start_p ze stala predkoscia katowa wokol srodka elipsy
    x = start_p.x
    y = start_p.y
    z = start_p.z
    z_bottom = start_p.z
    z_top = 0.4


    # parametry elipsy:
    a = 0.3
    b = (z_top - z_bottom) / 2

    freq = 30
    T = 4
    dT = 1. / freq

    dFi = 6.283185 / (T * freq)
    Fi = 0

    dT_number = T * freq

    move_within_start_points('forward')
    
    err = 0.01
    for i in range(0, dT_number * times):
        Fi += dFi
        z = sqrt( (b**2) / (1 +  (b**2 * tan(Fi)**2) / (a**2) ) )

        #zabezpieczenie przed bledami wynikajacymi z mnozenia 0 * oo:
        if z < err:
            continue

        if i % dT_number <= dT_number * 0.25 or i % dT_number >= dT_number * 0.75:
            y = - z * tan(Fi)
            z = z_bottom + b - z
        else:
            y = z * tan(Fi)
            z = z_bottom + b + z

        
        if i == T * freq * 0.25:
            y = -a
        if i == T * freq * 0.75:
            y = a

        interpolation = rospy.ServiceProxy('/oint_control_srv', OintServiceStruct)
        interpolation(x, y, z, 0, 0, 0, 1, dT, 'linear')

        if i == T * freq:
            Fi = 0

    move_within_start_points('backward')


if __name__ == "__main__":
    rospy.init_node('shape_drawer')
    if len(sys.argv) != 3 or (sys.argv[1] != 'rectangle' and sys.argv[1] != 'elipse'):
        rospy.logerr("Invalid call. Program should be called as:")
        rospy.logerr(str(sys.argv[0]) + " shape times where \'shape\' is \'rectanlge\' or \'elipse\' and \'times\' is a number.")
        exit(1)
    
    try:
        times = int(sys.argv[2])
    except ValueError:
        rospy.logerr("Invalid call. Second argument \'times\' should be a number.")
        exit(1)

    if sys.argv[1] == 'rectangle':
        draw_rectangle(times)
    else:
        draw_elipse(times)
