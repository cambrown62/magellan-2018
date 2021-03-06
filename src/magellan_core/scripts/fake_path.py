#!/usr/bin/env python
import math
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler
from math import degrees

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion


class FakePathNode(object):

    def __init__(self):
        self._path = rospy.get_param('~path')
        self._step_size = rospy.get_param('~step_size')
        self._frame_id = rospy.get_param('~path_frame')
        self._publisher = rospy.Publisher('/path', Path, queue_size=5)

    def publish_path(self):
        if len(self._path) < 2:
            rospy.logerr('ERROR: too few points in path!')
            raise RuntimeError('invalid path provided')

        path = list(self._path)
        path_msg = Path()
        path_msg.header.frame_id = self._frame_id

        last = np.array(path.pop(0))

        while len(path) != 0:
            curr = np.array(path.pop(0))

            num_steps = math.ceil(np.linalg.norm(last-curr)/self._step_size)

            plan = zip(np.linspace(last[0], curr[0], num_steps), np.linspace(last[1], curr[1], num_steps))

            #path_heading = quaternion_from_euler(0, 0, np.arctan2(curr[0]-last[0], curr[1]-last[1]))
            path_heading = quaternion_from_euler(0, 0, np.arctan2(curr[1]-last[1], curr[0]-last[0]))
            path_heading = Quaternion(path_heading[0], path_heading[1], path_heading[2], path_heading[3])

            for point in plan:
                pose_msg = PoseStamped()
                pose_msg.pose.position.x = point[0]
                pose_msg.pose.position.y = point[1]

                pose_msg.pose.orientation = path_heading
                path_msg.poses.append(pose_msg)

            last = curr

        self._publisher.publish(path_msg)


if __name__ == '__main__':
    rospy.init_node('fake_path')

    node = FakePathNode()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        node.publish_path()
        rate.sleep()
