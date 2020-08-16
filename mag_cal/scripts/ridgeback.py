#!/usr/bin/env python

import rosbag

import datetime
from tf.msg import tfMessage
from argparse import ArgumentParser
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import MagneticField
from numpy import mean, array, hypot, diff, convolve, arange, sin, cos, ones, pi, matrix
from tf.transformations import euler_from_quaternion,quaternion_from_euler,quaternion_multiply,quaternion_matrix
import tf

# Prompt user if scipy is missing.
try:
  from scipy import optimize
except ImportError:
  print("This script requires scipy be available.")
  print("On Ubuntu: sudo apt-get install python-scipy")
  exit(1)


vecs = []
bag = rosbag.Bag("mag.bag")
for topic, msg, time in bag.read_messages(topics=("/imu/mag_fixed",)):
  vecs.append((float(msg.magnetic_field.x), float(msg.magnetic_field.y), float(msg.magnetic_field.z)))

print ("Using " + str(len(vecs)) + " samples.")

def calc_R(xc, yc):
    """ calculate the distance of each 2D points from the center (xc, yc) """
    return hypot(x-xc, y-yc)

def f_2(c):
    """ calculate the algebraic distance between the 2D points and the mean circle centered at c=(xc, yc) """
    Ri = calc_R(*c)
    return Ri - Ri.mean()


x,y,z = zip(*vecs)
center_estimate = mean(x), mean(y)
center, ier = optimize.leastsq(f_2, center_estimate)
radius = calc_R(*center).mean()
center = (center[0], center[1], mean(z))

a = arange(0, 2*pi + pi/50, pi/50)
circle_points = (center[0] + cos(a) * radius,
                 center[1] + sin(a) * radius,
                 center[2] * ones(len(a)))

print("Magnetic circle centered at " + str(center) + ", with radius " + str(radius))
