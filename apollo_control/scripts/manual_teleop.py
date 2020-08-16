#!/usr/bin/env python
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray, Bool, Float64
from minesweeper_msgs.msg import ControllerStats
import rospy
import math

class Controller(object):
    def __init__(self):
        rospy.init_node("manual_controller", anonymous=True)
        self._sub = rospy.Subscriber("joy", Joy, self.callback)
        self._timeout_sub = rospy.Subscriber("joy_timeout", Bool, self.timeout_cb)
        self._rate = rospy.Rate(50)
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self._fork_pub = rospy.Publisher("/fork_position_controller/command", Float64, queue_size=10)
        self._velocity = Twist()
        self._left_stick_y = 0.0
        self._left_stick_x = 0.0
        self._right_stick_x = 0.0
        self._right_stick_y = 0.0
        self._r2 = 0.0
        self._l2 = 0.0
        self._r22 = 0.0
        self._l22 = 0.0
        self._opt_button = 0
        self._l1 = 0
        self._r1 = 0
        self._profile_2_linear_axis = 0.0
        self._linear_x_speed_limit = rospy.get_param("teleop/linear_x_speed_limit", 1.50)
        self._linear_x_speed_step = rospy.get_param("teleop/linear_x_speed_step", 0.10)
        self._linear_y_speed_limit = rospy.get_param("teleop/linear_y_speed_limit", 1.50)
        self._linear_y_speed_step = rospy.get_param("teleop/linear_y_speed_step", 0.10)
        self._angular_speed_limit = rospy.get_param("teleop/angular_speed_limit", 1.00)
        self._angular_speed_step = rospy.get_param("teleop/angular_speed_step", 0.05)
        self._left_stick_deadzone = rospy.get_param("teleop/left_stick_deadzone", 0.05)
        self._right_stick_deadzone = rospy.get_param("teleop/right_stick_deadzone", 0.05)
        self._bluetooth_enabled = rospy.get_param("bluetooth", False)
        self._max_linear_x_speed = 1.0
        self._max_linear_y_speed = 1.0
        self._max_angular_speed = 1.0
        self._dpad_y = 0.0
        self._dpad_x = 0.0
        self._inverted_right_stick_x = -1  # for profile 1
        self._inverted_left_stick_x = 1  # for profile 2
        self._inverted_left_stick_crane = 1 # for crane profile
        self._r3 = 1
        self._l3 = 1
        self._share_button = 0
        self._square = 0
        self._circle = 0
        self._a = 0
        self._b = 0
        self._x = 0
        self._y = 0
        self._profile = 1
        self._linear_lock_x = False
        self._linear_lock_y = False
        self._inverted_angular_lock = False
        self._angular_lock = False
        self._share_lock = False
        self._profile_2_linear_lock = True
        self._opt_button_lock = False
        self._fork_lock = False

        self._bumper_axis = 0
        self._fork_states = rospy.get_param("teleop/fork_states", (0.0, 0.1, 0.15, 0.2, 0.3))
        self._fork_state = 0

        self._timeout_state = True

    def callback(self, data):
        self._left_stick_y = data.axes[1]
        self._left_stick_x = data.axes[0] * self._inverted_left_stick_x
        self._right_stick_x = data.axes[2] * self._inverted_right_stick_x
        self._right_stick_y = data.axes[3]
        self._r2 = data.axes[4]
        self._l2 = data.axes[5]
        self._dpad_y = data.axes[7]
        self._dpad_x = data.axes[6] * -1
        self._r3 = data.buttons[14]
        self._l3 = data.buttons[13]
        self._share_button = data.buttons[8]
        self._opt_button = data.buttons[12]
        self._l1 = data.buttons[6]
        self._r1 = data.buttons[7]
        self._a = data.buttons[0]
        self._b = data.buttons[1]
        self._x = data.buttons[3]
        self._y = data.buttons[4]

        if self._l1 and not self._r1:
            self._bumper_axis = -1
        elif not self._l1 and self._r1:
            self._bumper_axis = 1
        else:
            self._bumper_axis = 0

        self._fork_pub.publish(self._fork_states[self._fork_state])

    def timeout_cb(self, data):
        self._timeout_state = data.data

    def profile_1(self):
        self.speed_limits()
        if -self._left_stick_deadzone <= self._left_stick_x <= self._left_stick_deadzone:
            self._left_stick_x = 0.0
        if -self._left_stick_deadzone <= self._left_stick_y <= self._left_stick_deadzone:
            self._left_stick_y = 0.0

        if -self._right_stick_deadzone <= self._right_stick_x <= self._right_stick_deadzone:
            self._right_stick_x = 0.0
        if self._r3 != 0:
            if not self._inverted_angular_lock:
                self._inverted_right_stick_x = self._inverted_right_stick_x * -1
                self._inverted_angular_lock = True
        else:
            self._inverted_angular_lock = False

        if self._y != 0 or self._a != 0:
            if not self._fork_lock:
                if self._a == 1:
                    self._fork_state -= 1
                if self._y == 1:
                    self._fork_state += 1
                if self._fork_state <= 0:
                    self._fork_state = 0
                elif self._fork_state >= len(self._fork_states) - 1:
                    self._fork_state = len(self._fork_states) - 1
                self._fork_lock = True
        else:
            self._fork_lock = False

        self._velocity.linear.x = self._left_stick_y * self._max_linear_x_speed
        self._velocity.linear.y = self._left_stick_x * self._max_linear_y_speed
        self._velocity.angular.z = self._right_stick_x * self._max_angular_speed

    def profile_2(self):
        self.speed_limits()
        self._r22 = 1.0 + (((self._r2 - (-1.0))/(1.0 - (-1.0))) * (0.0 - 1.0))
        self._l22 = -1.0 + (((self._l2 - (-1.0)) / (1.0 - (-1.0))) * (1.0 - 0.0))
        if -0.05 <= self._left_stick_x <= 0.05:
            self._left_stick_x = 0.0
        if self._l3 != 0:
            if not self._inverted_angular_lock:
                self._inverted_left_stick_x = self._inverted_left_stick_x * -1
                self._inverted_angular_lock = True
        else:
            self._inverted_angular_lock = False
        if self._profile_2_linear_lock:
            if self._r2 != 0.0:
                self._profile_2_linear_axis = self._r22
            if self._l2 != 0.0:
                self._profile_2_linear_axis = self._l22
        if self._r2 and self._l2:
            self._profile_2_linear_lock = False
        if not self._profile_2_linear_lock:
            self._profile_2_linear_axis = self._l22 + self._r22

        self._velocity.linear.x = self._profile_2_linear_axis * self._max_linear_speed
        self._velocity.angular.z = self._left_stick_x * self._max_angular_speed

    def speed_limits(self):
        if self._dpad_y != 0.0:
            if self._linear_x_speed_step <= self._max_linear_x_speed <= self._linear_x_speed_limit:
                if not self._linear_lock_x:
                    self._max_linear_x_speed += self._dpad_y * self._linear_x_speed_step
                    self._linear_lock_x = True
            else:
                if self._max_linear_x_speed > self._linear_x_speed_limit:
                    self._max_linear_x_speed = self._linear_x_speed_limit
                elif self._max_linear_x_speed < self._linear_x_speed_step:
                    self._max_linear_x_speed = self._linear_x_speed_step
        else:
            self._linear_lock_x = False
        if self._dpad_x != 0.0:
            if self._linear_y_speed_step <= self._max_linear_y_speed <= self._linear_y_speed_limit:
                if not self._linear_lock_y:
                    self._max_linear_y_speed += self._dpad_x * self._linear_y_speed_step
                    self._linear_lock_y = True
            else:
                if self._max_linear_y_speed > self._linear_y_speed_limit:
                    self._max_linear_y_speed = self._linear_y_speed_limit
                elif self._max_linear_y_speed < self._linear_y_speed_step:
                    self._max_linear_y_speed = self._linear_y_speed_step
        else:
            self._linear_lock_y = False

        if self._bumper_axis != 0.0:
            if self._angular_speed_step <= self._max_angular_speed <= self._angular_speed_limit:
                if not self._angular_lock:
                    self._max_angular_speed += self._bumper_axis * self._angular_speed_step
                    self._angular_lock = True
            else:
                if self._max_angular_speed > self._angular_speed_limit:
                    self._max_angular_speed = self._angular_speed_limit
                elif self._max_angular_speed < self._angular_speed_step:
                    self._max_angular_speed = self._angular_speed_step
        else:
            self._angular_lock = False

    def stop(self):
        if self._x == 1 or not self._timeout_state:
            self._velocity.linear.x = 0.0
            self._velocity.linear.y = 0.0
            self._velocity.angular.z = 0.0

    def publisher(self):
        while not rospy.is_shutdown():
            if self._share_button == 1:
                if not self._share_lock:
                    if self._profile == 1:
                        self._profile = 2
                    elif self._profile == 2:
                        self._profile = 1
                    elif self._profile == 3:
                        self._profile = 1
                    self._velocity.linear.x = 0.0
                    self._velocity.linear.y = 0.0
                    self._velocity.angular.z = 0.0
                    self._share_lock = True
            else:
                self._share_lock = False

            if self._opt_button == 1:
                if not self._opt_button_lock:
                    if self._profile != 3:
                        self._profile = 3
                    else:
                        self._profile = 1
                    self._opt_button_lock = True
            else:
                self._opt_button_lock = False

            if self._profile == 1:
                self.profile_1()
            #elif self._profile == 2:
            #    self.profile_2()

            self.stop()
            self._pub.publish(self._velocity)
            #self._fork_pub.publish(self._fork_states[self._fork_state])
            self._rate.sleep()

def main():
    controller = Controller()
    controller.publisher()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
