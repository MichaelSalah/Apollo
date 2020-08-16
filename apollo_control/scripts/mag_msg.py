#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import MagneticField

class Relay:
    def __init__(self):
        rospy.init_node("relay")
        self._pub = rospy.Publisher("imu/mag", MagneticField, queue_size=10)
        self._rate = rospy.Rate(50)
        self._sub = rospy.Subscriber("imu/mag_raw", Vector3Stamped, self.cb)
        self._msg = MagneticField()
        self._msg.magnetic_field_covariance = (100, 0.0, 0.0, 100, 0.0, 0.0, 100, 0.0, 0.0)

    def cb(self, data):
        self._msg.header = data.header
        self._msg.magnetic_field = data.vector
        self._pub.publish(self._msg)

    def run(self):
        rospy.spin()

def main():
    while not rospy.is_shutdown():
        try:
            r = Relay()
            r.run()
        except ros.ROSInterruptException:
            pass

if __name__ == "__main__":
    main()
