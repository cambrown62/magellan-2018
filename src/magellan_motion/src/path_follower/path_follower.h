#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include "path_markers.h"
#include "velocity_limiter.h"

class PathFollower {
public:
    PathFollower(ros::NodeHandle& nh, double discretization, double max_vel, double max_acc, double kP, double kD);
    void Update();
private:
    double kP_;
    double kD_;
    ros::NodeHandle& nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Subscriber path_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Publisher actuator_publisher_;
    ros::Publisher cross_track_error_publisher_;
    ros::Publisher point_pose_publisher_;

    PathMarkers markers_;
    VelocityLimiter velocity_limiter_;

    nav_msgs::Path::ConstPtr current_path_;
    nav_msgs::Odometry::ConstPtr current_odom_;
    int path_start_index_;

    double discretization_;
    double max_vel_;
    double max_acc_;

    void UpdatePath(nav_msgs::Path::ConstPtr path);
    void UpdateOdom(nav_msgs::Odometry::ConstPtr odom);
};

#endif
