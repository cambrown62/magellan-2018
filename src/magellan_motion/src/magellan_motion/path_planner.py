#!/usr/bin/env python

import rospy
import heapq
import math

from nav_msgs.msg import Path


class PathPlannerException(RuntimeError):
    pass


class MotionQueue(object):
    def __init__(self, start_list):
        heapq.heapify(start_list)
        self._queue = start_list

    def insert(self, item):
        heapq.heappush(self._queue, item)

    def pop(self):
        return heapq.heappop(self._queue)

    def is_empty(self):
        return len(self._queue) == 0


class PathPlanner(object):
    # 2D Path planner. Uses A* Path Planning to deterimine
    # trajectory from start to goal
    # nodes are represented as a list, i.e. [cost, x, y]
    # start and end goal are represented in list as [x, y]
    def __init__(self):
        _settings = rospy.get_param('~planner_settings')
        self._discret = _settings['discret']
        self._max_num = _settings['max_num']
        self._max_planning_dist = _settings['max_planning_dist']
        self._max_abs_theta = _settings['max_theta']

        self._goal = None
        self._queue = MotionQueue([])

    def _is_goal(self, node):
        return node[1]==self._goal[0] and node[2]==self._goal[1]

    def _get_cost(self, pose):
        return math.sqrt((pose[0]-self._goal[0])^2 + (pose[1]-self._goal[1])^2)

    def plan(self, start):
        if self._goal is None or start is None:
            raise PathPlannerException('Goal or Start to plan to')

        if self._is_goal(start):
            rospy.logerr('Goal and start state are the same!')
            return False, Path()

        if self._is_in_obstacle(start):
            raise PathPlannerException('START IS IN AN OBSTACLE!')

        first = [_get_cost(start)]
        first.append(start)
        self._queue.insert(first)

        # passed initial checks, now plan
        while not rospy.is_shutdown() and not self._queue.is_empty():
            curr_node = self._queue.pop()

            if self._is_goal(curr_node):
                rospy.loginfo('Found Goal!')
                break

        return True, Path()

    def _is_in_obstacle(self, pose):
        for obst in self._obstacles:
            pass
        return False

    def reset(self, goal):
        self._goal = goal
        return True
