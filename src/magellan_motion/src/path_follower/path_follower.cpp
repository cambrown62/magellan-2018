#include "path_follower.h"
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <magellan_core/ActuatorCommand.h>
#include <optional>

static inline double L2Norm(const geometry_msgs::PoseStamped& pose) {
    return std::sqrt(pose.pose.position.x * pose.pose.position.x + pose.pose.position.y * pose.pose.position.y);
}

static inline double L2Norm(const geometry_msgs::Vector3& vector) {
    return std::sqrt(vector.x * vector.x + vector.y * vector.y);
}

PathFollower::PathFollower(ros::NodeHandle& nh,
                           double discretization,
                           double max_vel,
                           double max_acc,
                           double kP,
                           double kD) :
        kP_(kP),
        kD_(kD),
        nh_(nh),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        path_subscriber_(nh.subscribe("/path", 10, &PathFollower::UpdatePath, this)),
        odom_subscriber_(nh.subscribe("/odometry/filtered", 10, &PathFollower::UpdateOdom, this)),
        actuator_publisher_(nh.advertise<magellan_core::ActuatorCommand>("/platform/cmd_actuators", 10)),
        cross_track_error_publisher_(nh.advertise<std_msgs::Float64>("/path/cross_track_error", 50)),
        point_pose_publisher_(nh.advertise<geometry_msgs::PoseStamped>("/path/point_pose", 50)),
        markers_(nh, 10),
        velocity_limiter_(max_vel, max_acc),
        current_path_(),
        current_odom_(),
        path_start_index_(0),
        discretization_(discretization),
        max_acc_(max_acc) {
            ROS_WARN("Started kp= %2.2f kd= %2.2f", kP_, kD_);
}

void PathFollower::UpdatePath(nav_msgs::Path::ConstPtr path) {
    current_path_ = path;
    path_start_index_ = 0;
}

void PathFollower::UpdateOdom(nav_msgs::Odometry::ConstPtr odom) {
    current_odom_ = odom;
}

void PathFollower::Update() {
    if ( !current_path_ ) {
        ROS_WARN("no path");
        return;
    }

    if ( !current_odom_ ) {
        ROS_WARN("no path");
        return;
    }

    if ( current_path_->poses.size() == 0 ) {
        ROS_WARN("Commanded path is empty");
        return;
    }

    magellan_core::ActuatorCommand command;
    auto transform = tf_buffer_.lookupTransform("base_link",
                                                current_path_->header.frame_id,
                                                ros::Time::now(),
                                                ros::Duration(1.0));

    auto it = current_path_->poses.begin();
    geometry_msgs::PoseStamped closest_point;
    tf2::doTransform(*it, closest_point, transform);
    geometry_msgs::PoseStamped temp;
    for ( it += 0; it != current_path_->poses.end(); ++it) {
        tf2::doTransform(*it, temp, transform);
        if ( L2Norm(temp) > L2Norm(closest_point) )
            break;
        closest_point = temp;
    }
    double cross_track_error = L2Norm(closest_point);
    path_start_index_ = std::distance(current_path_->poses.begin(), it) - 1;
    double lookahead_distance_ = 0.5 + std::max(cross_track_error, 1.0);
    int lookahead_points = (lookahead_distance_ + cross_track_error) / discretization_;
    int points_to_end = std::distance(it, current_path_->poses.end());
    it += std::min(points_to_end - 1, lookahead_points);

    geometry_msgs::PoseStamped lookahead_pose;
    tf2::doTransform(*it, lookahead_pose, transform);

    double linear_velocity = L2Norm(current_odom_->twist.twist.linear);

    command.steering_angle = std::copysign(lookahead_pose.pose.position.x * kP_, -lookahead_pose.pose.position.y);


    double estimated_remaining_distance = points_to_end * discretization_;

    double speed = 0;
    if ( estimated_remaining_distance > 0 )
        speed = max_vel_;
    command.velocity = speed;

    actuator_publisher_.publish(command);
    markers_.Update(closest_point, lookahead_pose);

    std_msgs::Float64 cross_track_error_msg;
    cross_track_error_msg.data = cross_track_error;
    cross_track_error_publisher_.publish(cross_track_error_msg);
}


