#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField, Imu

class Relay:
    def __init__(self):
        rospy.init_node("imu_time")
        self._imu_pub = rospy.Publisher("imu/data_fixed", Imu, queue_size=10)
        self._mag_pub = rospy.Publisher("imu/mag_fixed", MagneticField, queue_size=10)
        self._rate = rospy.Rate(50)
        self._imu_sub = rospy.Subscriber("android/imu", Imu, self.imu_cb)
        self._mag_sub = rospy.Subscriber("android/magnetic_field", MagneticField, self.mag_cb)
        self._imu_msg = Imu()
        self._mag_msg = MagneticField()

    def imu_cb(self, data):
        self._imu_msg.orientation = data.orientation
        self._imu_msg.orientation_covariance = [10000.0, 0.0, 0.0,
                                                0.0, 10000.0, 0.0,
                                                0.0, 0.0,  0.0001]
        self._imu_msg.angular_velocity = data.angular_velocity
        self._imu_msg.angular_velocity_covariance = [10000.0, 0.0, 0.0,
                                                     0.0, 10000.0, 0.0,
                                                     0.0, 0.0,  0.0001]
        self._imu_msg.linear_acceleration = data.linear_acceleration
        self._imu_msg.linear_acceleration_covariance = [0.0001, 0.0, 0.0,
                                                        0.0, 0.0001, 0.0,
                                                        0.0, 0.0,  10000.0]

        self._imu_msg.header.stamp = rospy.get_rostime()
        self._imu_msg.header.frame_id = "/imu"
        self._imu_pub.publish(self._imu_msg)

    def mag_cb(self, data):
        self._mag_msg.magnetic_field = data.magnetic_field
        self._mag_msg.magnetic_field_covariance = [0.0001, 0.0, 0.0,
                                                   0.0, 0.0001, 0.0,
                                                   0.0, 0.0, 0.0001]
        self._mag_msg.header.stamp = rospy.get_rostime()
        self._mag_msg.header.frame_id = "/imu"
        self._mag_pub.publish(self._mag_msg)

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
