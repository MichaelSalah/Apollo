#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3Stamped

class Relay:
    def __init__(self):
        rospy.init_node("imu_time")
        self._mag_pub = rospy.Publisher("imu/mag_fixed", MagneticField, queue_size=10)
        self._mag_sub = rospy.Subscriber("imu/mag_raw", Vector3Stamped, self.mag_cb)
        self._mag_msg = MagneticField()

    def mag_cb(self, data):
        self._mag_msg.magnetic_field = data.vector
        self._mag_msg.magnetic_field_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
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
