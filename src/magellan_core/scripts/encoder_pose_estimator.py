#!/usr/bin/env python
from threading import RLock
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance
from magellan_core.msg import EncoderDeltaStamped

class EncoderPoseEstimator:
    def __init__(self):
        self._lock = RLock()
        with self._lock:
            self._odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self._update_robot_localization_odom)
            self._odom_pub = rospy.Publisher("/odometry/encoders", Odometry, queue_size=3)
            self._encoder_sub = rospy.Subscriber("/platform/encoders", EncoderDeltaStamped, self._update_encoder_delta)

            self._encoder_cpr = 10
            self._wheel_diameter = 0.075
            self._one_tick_distance = (pi*self._wheel_diameter) / self._encoder_cpr

            self._turn_radius_sub = rospy.Subscriber("/platform/turning_radius", Float64, self._update_turning_radius)
            self._robot_localization_odom = Odometry()
            self._turning_radius = 0
            self._odom_covariance = 0.1
            self._world_position = [0, 0]

    def _update_robot_localization_odom(self, odom):
        with self._lock:
            self._robot_localization_odom = odom

    def _update_turning_radius(self, radius):
        with self._lock:
            self._turning_radius = radius

    def _update_encoder_delta(self, encoder_delta):
        with self._lock:
            new_odom = Odometry()
            new_odom.header.frame_id = 'odom'
            new_odom.child_frame_id = 'base_link'

            avg_displacement = self._one_tick_distance * ((encoder_delta.left_delta + encoder_delta.right_delta) / 2.0)

            # Find heading from orientation quaternion
            q = self._robot_localization_odom.pose.pose.orientation
            heading = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

            robot_delta = [0, 0]
            if self._turning_radius == 0:
                robot_delta[0] = avg_displacement
            else:
                pass

            world_delta = [
                robot_delta[0]*cos(heading) - robot_delta[1]*sin(heading),
                robot_delta[0]*sin(heading) + robot_delta[1]*cos(heading)
            ]

            self._world_position[0] += world_delta[0]
            self._world_position[1] += world_delta[1]

            new_odom.header.stamp = rospy.Time.now()
            new_odom.pose.covariance[0] = self._odom_covariance
            new_odom.pose.covariance[7] = self._odom_covariance

            new_odom.pose.pose.position.x = self._world_position[0]
            new_odom.pose.pose.position.y = self._world_position[1]

            self._odom_pub.publish(new_odom)

rospy.init_node('encoder_pose_estimator')
node = EncoderPoseEstimator()
rospy.spin()
