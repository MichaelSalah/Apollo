#!/usr/bin/env python
import rospy
import numpy as np
import random
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Vector3, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker

class Tag:
    def __init__(self, location, radius, variance=0):
        self._location = location
        self._radius = radius
        self._variance = variance if variance != 0 else (radius**2) / 9
        self._msg = PoseWithCovarianceStamped()
        self._msg.header.frame_id = "map"
        self._msg.pose.pose.orientation.w = 1.0
        self._msg.pose.covariance = np.identity(6)
        self._msg.pose.covariance[0][0] = self._msg.pose.covariance[1][1] = self._variance
        self._msg.pose.covariance = self._msg.pose.covariance.reshape(1, 36).tolist()[0]

    def check_if_activated(self, robot_location):
        circle = (robot_location.x - self._location[0]) ** 2 + (robot_location.y - self._location[1]) ** 2
        if circle <= self._radius ** 2:
            self._msg.pose.pose.position.x, self._msg.pose.pose.position.y = self._location
            self._msg.header.stamp = rospy.Time.now()
            if not rospy.is_shutdown():
                return self._msg
        else:
            return None

    def get_data(self):
        return tuple([self._location, self._radius])


class Indoor:
    def __init__(self, robot_name="apollo", tags=(), radius=0.02, variance=0):
        rospy.init_node("indoor_localization")
        self._sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.cb)
        self._publisher = rospy.Publisher("/rfid_pose", PoseWithCovarianceStamped, queue_size=1)
        self._rate = rospy.Rate(1)
        self._first_msg_received = False
        self._index_of_robot = 0
        self._msg = PoseWithCovarianceStamped()
        self._robot_name = robot_name
        self._tags = [self.create_tag(x, radius, variance) for x in tags]
        self._robot_location = Pose()
        self._publish_once = False
        self._radius = radius
        self._markers_publisher = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=100, latch=True)
        self._markers = MarkerArray()


    def cb(self, data):
        if not self._first_msg_received:
            try:
                print(data.name)
                self._index_of_robot = list(data.name).index(self._robot_name)
                self._first_msg_received = True
            except ValueError:
                self._first_msg_received = False
        self._robot_location = data.pose[self._index_of_robot]

        for tag in self._tags:
            self._msg = tag.check_if_activated(self._robot_location.position)
            if self._msg:
                break
        else:
            self._publish_once = False

        if not self._publish_once and self._msg:
            self._publisher.publish(self._msg)
            #rospy.sleep(1)
            #self._rate.sleep()
            self._publish_once = True

    def create_tag(self, location, radius=0.02, variance=0):
        return Tag(location, radius, variance)

    def pub_markers(self):
        markers = []
        for i, tag in enumerate(self._tags):
            data = tag.get_data()
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "tags"
            marker.id = i
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.pose.position = Point(data[0][0], data[0][1], 0)
            marker.pose.orientation = Quaternion(0, 0, 0, 1.0)
            marker.scale = Vector3(self._radius * 2, self._radius * 2, 0.01)
            marker.color = ColorRGBA(random.random(), random.random(), random.random(), 1)
            marker.lifetime = rospy.Duration(0)
            markers.append(marker)
        self._markers_publisher.publish(MarkerArray(markers))

    def start_tracking(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        tags = rospy.get_param("indoor_localization/tags")
        i = Indoor(tags=tags)
        i.pub_markers()
        i.start_tracking()

    except rospy.ROSInterruptException:
        pass
