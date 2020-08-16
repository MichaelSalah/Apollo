#!/usr/bin/env python
import rosbag
from sensor_msgs.msg import MagneticField
import numpy as np
from scipy import linalg
from argparse import ArgumentParser

bag = rosbag.Bag("both.bag")
c_samples = []
nc_samples = []
for topic, msg, time in bag.read_messages(topics="/imu/mag_fixed"):
    nc_samples.append((float(msg.magnetic_field.x), float(msg.magnetic_field.y), float(msg.magnetic_field.z)))

for topic, msg, time in bag.read_messages(topics="/imu/mag"):
    c_samples.append((float(msg.magnetic_field.x), float(msg.magnetic_field.y), float(msg.magnetic_field.z)))

cx,cy,cz = zip(*c_samples)
ncx,ncy,ncz = zip(*nc_samples)

print("non-calibrated:\n\t{} {}".format(np.mean(ncx), np.mean(ncy)))
print("calibrated:\n\t{} {}".format(np.mean(cx), np.mean(cy)))
