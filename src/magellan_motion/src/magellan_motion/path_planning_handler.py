#!/usr/bin/env python

import rospy
import threading

from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Path, Odometry
from magellan_core.msg import WaypointStamped

from path_planner import PathPlanner


class HandlerException(RuntimeError):
    pass


class PathPlanningHandler(object):
    def __init__(self, path_planner):
        self._lock = threading.RLock()

        self._goal_sub = rospy.Subscriber('/waypoint', WaypointStamped, self._handle_new_goal)
        self._odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self._handle_odom)
        self._path_pub = rospy.Publisher('/path', Path, queue_size=10)

        self._new_goal_available = False

        self._goal = None
        self._odom = None

    def run(self):
        with self._lock:
            if self._goal is None:
                rospy.loginfo('Planner Handler waiting for goal')
                return

            if self._new_goal_available and self._odom is not None:
                _goal_point = self._goal.waypoint.goal.position
                # assume 0 orientation
                goal = (_goal_point.x, _goal_point.y, 0.0)

                _x = self._odom.pose.pose.position.x
                _y = self._odom.pose.pose.position.y

                _start_quaternion = (
                    self._odom.pose.orientation.x,
                    self._odom.pose.orientation.y,
                    self._odom.pose.orientation.z,
                    self._odom.pose.orientation.w)

                # roll and pitch should be 0
                roll, pitch, yaw = euler_from_quaternion(_start_quaternion)
                # yaw will be the robot's orientation
                start = (_x, _y, yaw)

                status = self._planner.reset(start, goal)

                if not status:
                    raise ('FAILED TO RESET GS PLANNER!')
                else:
                    self._new_goal_available = False

                try:
                    success, path = self._planner.plan()
                except Exception:
                    rospy.logerr('PLANNER THREW EXCEPTION')
                    success = False

                if not success:
                    rospy.logerr('PLANNER FAILED TO FIND A PATH')
                    return

                self._path_pub.publish(path)
            else:
                rospy.logwarn('No odom or goal available')

    def _handle_new_goal(self, msg):
        with self._lock:
            self._goal = msg
            self._new_goal_available = True

    def _handle_odom(self, msg):
        with self._lock:
            self._odom = msg


if __name__ == '__main__':
    rospy.init_node('path_planner')

    _rate = rospy.Rate(rospy.get_param('~handler_update_rate'))

    _planner = PathPlanner()
    _handler = PathPlanningHandler(_planner)

    while not rospy.is_shutdown():
        _handler.run()
        _rate.sleep()
