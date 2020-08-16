#!/usr/bin/env python
import rosbag
from sensor_msgs.msg import MagneticField
import numpy as np
from scipy import linalg
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("bag", type=str, metavar="FILE")
args = parser.parse_args()

bag = rosbag.Bag(args.bag)

samples = []

def elipsoid_fit(s):
    D = np.array([s[0] ** 2., s[1] ** 2., s[2] ** 2.,
                  2. * s[1] * s[2], 2. * s[0] * s[2], 2. * s[0] * s[1],
                  2. * s[0], 2. * s[1], 2. * s[2], np.ones_like(s[0])])

    # S, S_11, S_12, S_21, S_22 (eq. 11)
    S = np.dot(D, D.T)
    S_11 = S[:6, :6]
    S_12 = S[:6, 6:]
    S_21 = S[6:, :6]
    S_22 = S[6:, 6:]

    # C (Eq. 8, k=4)
    C = np.array([[-1, 1, 1, 0, 0, 0],
                  [1, -1, 1, 0, 0, 0],
                  [1, 1, -1, 0, 0, 0],
                  [0, 0, 0, -4, 0, 0],
                  [0, 0, 0, 0, -4, 0],
                  [0, 0, 0, 0, 0, -4]])

    # v_1 (eq. 15, solution)
    E = np.dot(linalg.inv(C),
               S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

    E_w, E_v = linalg.eig(E)

    v_1 = E_v[:, np.argmax(E_w)]
    if v_1[0] < 0: v_1 = -v_1

    # v_2 (eq. 13, solution)
    v_2 = np.dot(np.dot(-linalg.inv(S_22), S_21), v_1)

    # quadric-form parameters
    M = np.array([[v_1[0], v_1[3], v_1[4]],
                  [v_1[3], v_1[1], v_1[5]],
                  [v_1[4], v_1[5], v_1[2]]])
    n = np.array([[v_2[0]],
                  [v_2[1]],
                  [v_2[2]]])
    d = v_2[3]

    return M, n, d


def main():
    F = 4.37559e-5
    for topic, msg, time in bag.read_messages(topics="/imu/mag_fixed"):
        samples.append((float(msg.magnetic_field.x), float(msg.magnetic_field.y), float(msg.magnetic_field.z)))

    s = np.array(samples).T
    M, n, d = elipsoid_fit(s)
    M_1 = linalg.inv(M)
    b = -np.dot(M_1, n)
    A_1 = np.real(F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) *
                       linalg.sqrtm(M))

    print(b)
    print(A_1)

main()
