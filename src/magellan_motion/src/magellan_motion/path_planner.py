#!/usr/bin/env python

import rospy
import heapq

from nav_msgs.msg import Path


class PathPlannerException(RuntimeError):
    pass


class MotionQueue(object):
    def __init__(self, start_list):
        self._queue = heapq.heapify(start_list)

    def insert(self, item):
        heapq.heappush(self._queue, item)

    def pop(self):
        return heapq.heappop(self._queue)


class PathPlanner(object):
    def __init__(self):
        _settings = rospy.get_param('~planner_settings')
        self._discret = _settings['discret']
        self._max_num = _settings['max_num']
        self._max_planning_dist = _settings['max_planning_dist']
        self._max_abs_theta = _settings['max_theta']

        self._goal = None
        self._queue = MotionQueue()

    def plan(self, start):
        if self._goal is None or start is None:
            raise PathPlannerException('Goal or Start to plan to')

        if self._goal == start:
            rospy.logerr('Goal and start state are the same!')
            return False, Path()

        if self._is_in_obstacle(start):
            raise PathPlannerException('START IS IN AN OBSTACLE!')

        return True, Path()

    def _is_in_obstacle(self, primitive):
        for obst in self._obstacles:
            pass
        return False

    def reset(self, goal):
        self._goal = goal
        return True
