#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math


class Limiter():
    def __init__(self):
        rospy.init_node("velocity_limiter")
        self._pub = rospy.Publisher("/apollo_velocity_controller/cmd_vel", Twist, queue_size=1)
        self._sub = rospy.Subscriber("/cmd_vel", Twist, self.cb)
        self._twist = Twist()
        self._max_x = rospy.get_param("/apollo_velocity_controller/linear/x/max_velocity")
        self._max_y = rospy.get_param("/apollo_velocity_controller/linear/y/max_velocity")
        self._max_th = rospy.get_param("/apollo_velocity_controller/angular/z/max_velocity")

    def cb(self, data):
        x = data.linear.x
        y = data.linear.y
        th = data.angular.z
        self._twist.linear.x = math.copysign(min(abs(x), self._max_x), x)
        self._twist.linear.y = math.copysign(min(abs(y), self._max_y), y)
        self._twist.angular.z = math.copysign(min(abs(th), self._max_th), th)
        self._pub.publish(self._twist)

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass


def main():
    l = Limiter()
    l.run()


if __name__ == "__main__":
    main()

