#!/usr/bin/env python
import rospy
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import Empty

lock = True

def cb(data):
    global lock
    status = data.status_list
    if len(status) > 0 and status[0].status == 3 and not lock:
        rospy.sleep(1)
        clear_costmap()
        lock = True
    if status[0].status != 3:
        lock = False

def clear_costmap():
    rospy.wait_for_service('move_base/clear_costmaps')
    try:
        clear = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        clear()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main():
    rospy.init_node("goal_watch")
    rospy.Subscriber("move_base/status", GoalStatusArray, cb)
    rospy.spin()

if __name__ == "__main__":
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
