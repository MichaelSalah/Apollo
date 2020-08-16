#!/usr/bin/env python
import rospy
import smach
import smach_ros
from move_base_msgs.msg import *
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Range
import argparse

parser = argparse.ArgumentParser(description="Please put te following parameter to start this program")
parser.add_argument('-FM', '--free_mode', type=bool, default=False, metavar='', help="free mode allow you to open the move_base")
parser.add_argument('-FH', '--fork_height', type=int, default=1, metavar='', help="shelf level one = 1 shelf level two = 2")
parser.add_argument('-PR','--product', type=int, default=0, metavar='', help="please choose between product 'a'or 'b'")
parser.add_argument('-DZ','--drop_zone', type=str, default='a', metavar='', help="please choose a drop_zone 'a'or 'b'")
args = parser.parse_args()


class Fork(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["failed", "succeeded", "drop", "end", "wait"],
                             input_keys=["state", "goal", "target", "free"], output_keys=["direction", "target"])
        self.pub = rospy.Publisher("fork_position_controller/command", Float64, queue_size=1)
        self.sub = 0
        self.err = 0
        self.goal = 0
        self.increment = rospy.get_param('move_base/fork_increment', 0.02)
        self.time_delay = rospy.get_param("move_base/time_delay", 1)
        self.fork_error_tolerance = rospy.get_param("move_base/fork_error_tolerance", 0.0005)
        self.failure_time = rospy.get_param("move_base/failure_time", 20)
        self.end_state = False
        self.start_time = 0

    def execute(self, ud):
        if ud.state == 0:
            self.goal = ud.goal
        elif ud.state == 1:
            rospy.sleep(self.time_delay)
            self.goal += self.increment
        elif ud.state == 2:
            self.goal = self.increment
        if ud.free or self.end_state:
            return "end"

        self.start_time = rospy.get_time()
        self.sub = rospy.Subscriber("fork_position_controller/state", JointControllerState, self.fork_cb)
        self.pub.publish(self.goal)

        rospy.sleep(self.time_delay)
        if ud.state == 0:
            ud.direction = -1

        elif ud.state == 1:
            ud.direction = 1

        if ud.state == 2:
            self.end_state = True
            ud.target = PoseStamped()
            ud.target.header.frame_id = "map"
            """REMOVE WHEN GUI IS AVAILABLE"""
            if args.drop_zone=='a':
                ud.target.pose.position.x = 0.0  # args
                ud.target.pose.position.y = 0.0  # args
                ud.target.pose.orientation.w = 1.0  # args
            elif args.drop_zone=='b':
                ud.target.pose.position.x = 1.0  # args
                ud.target.pose.position.y = 1.0  # args
                ud.target.pose.orientation.w = 1.0  # args
            while self.err > self.fork_error_tolerance:
                pass
            else:
                return "drop"
        else:
            return "succeeded"

    def fork_cb(self, data):
        self.err = abs(data.error)
        if self.err > self.fork_error_tolerance:
            if rospy.get_time() - self.start_time > self.failure_time:
                return "failed"

        else:
            #self.pub.unregister()
            self.sub.unregister()


class Linear(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["failed", "succeeded"], input_keys=["direction", "state"],
                             output_keys=["state"])
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(50)
        self.sub = 0
        self.range = 0
        self.linear_speed = rospy.get_param("move_base/linear_speed", 0.1)
        self.retreat_distance = rospy.get_param("move_base/retreat_distance", 0.3)
        self.stop_distance = rospy.get_param("move_base/stop_distance", 0.06)
        self.time_delay = rospy.get_param("move_base/time_delay", 1)
        self.failure_time = rospy.get_param("move_base/failure_time", 20)
        self.start_time = 0

    def execute(self, ud):
        t = Twist()
        self.start_time = rospy.get_time()
        t.linear.x = ud.direction * self.linear_speed
        self.sub = rospy.Subscriber("rear_ultrasonic", Range, self.us_cb)
        rospy.sleep(self.time_delay)
        if ud.direction < 0:
            while self.range >= self.stop_distance:
                if rospy.get_time() - self.start_time > self.failure_time:
                    return "failed"
                self.pub.publish(t)
                self.rate.sleep()
        elif ud.direction > 0:
            while self.range <= self.retreat_distance:
                if rospy.get_time() - self.start_time > self.failure_time:
                    return "failed"
                self.pub.publish(t)
                self.rate.sleep()

        if ud.state == 0:
            ud.state = 1
        elif ud.state == 1:
            ud.state = 2

        return "succeeded"

    def us_cb(self, data):
        self.range = data.range


def result_cb(ud, status, result):
    if status == 3:
        pass

def main():
    rospy.init_node("nav_state_machine", anonymous=True)
    fork_levels = rospy.get_param('move_base/fork_levels',[0.03, 0.25])
    sm = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])
    sm.userdata.target_pose = PoseStamped()
    sm.userdata.target_pose.header.frame_id = "map"
    """REMOVE WHEN GUI IS AVAILABLE"""
    products = rospy.get_param("indoor_localization/tags")
    sm.userdata.target_pose.pose.position.x = products[args.product][0]  # args
    sm.userdata.target_pose.pose.position.y = products[args.product][1]  # args
    sm.userdata.target_pose.pose.orientation.w = 1.0  # args

    sm.userdata.fork_level = fork_levels[args.fork_height - 1]
    sm.userdata.state = 0
    sm.userdata.free_mode = args.free_mode

    with sm:
        smach.StateMachine.add("MOVE_BASE",
                               smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_slots=["target_pose"],
                                                           result_cb=result_cb),
                               transitions={"succeeded": "FORK", "aborted": "MOVE_BASE", "preempted": "preempted"},
                               remapping={"target_pose": "target_pose"})
        smach.StateMachine.add("FORK", Fork(),
                               transitions={"succeeded": "LIN", "failed": "aborted", "drop": "MOVE_BASE",
                                            "end": "succeeded", "wait": "FORK"},
                               remapping={"state": "state", "goal": "fork_level", "direction": "direction",
                                          "target": "target_pose", "free": "free_mode"})
        smach.StateMachine.add("LIN", Linear(), transitions={"succeeded": "FORK", "failed": "aborted"},
                               remapping={"direction": "direction", "state": "state"})

    sis = smach_ros.IntrospectionServer("move_base", sm, "SM_ROOT")
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()s::CallbackQueue
