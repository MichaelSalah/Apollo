#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import MagneticField

class Relay:
    def __init__(self):
        rospy.init_node("imu_cor")
        self._mag_pub = rospy.Publisher("imu/mag", MagneticField, queue_size=10)
        self._mag_sub = rospy.Subscriber("imu/mag_fixed", MagneticField, self.mag_cb)
        self._mag_msg = MagneticField()
        self._s = []
        self._A_1 = np.array([[ 1.40474654e+00, -1.01694908e-02, -5.46992262e-03],
                              [-1.01694908e-02,  1.40470196e+00, -1.20123691e-05],
                              [-5.46992262e-03, -1.20123691e-05,  1.99726589e+00]])
        self._B = np.array([[1.00562186e-05], [3.59293346e-06], [-3.05659071e-05]])

    def mag_cb(self, data):
        self._s = np.array([data.magnetic_field.x, data.magnetic_field.y, data.magnetic_field.z]).reshape((3, 1))
        self._s = np.dot(self._A_1, self._s - self._B).T
        self._mag_msg.magnetic_field.x = self._s[0, 0]
        self._mag_msg.magnetic_field.y = self._s[0, 1]
        self._mag_msg.magnetic_field.z = self._s[0, 2]
        self._mag_msg.magnetic_field_covariance = data.magnetic_field_covariance
        self._mag_msg.header.stamp = rospy.get_rostime()
        self._mag_pub.publish(self._mag_msg)

    def run(self):
        rospy.spin()

def main():
    while not rospy.is_shutdown():
        try:
            r = Relay()
            r.run()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    main()
