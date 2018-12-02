#include <ros/ros.h>
#include "path_follower.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double rate_hz = 100;
    private_nh.getParam("rate", rate_hz);
    ros::Rate rate(rate_hz);

    double discretization;
    if ( !private_nh.getParam("discretization", discretization) ) {
        ROS_ERROR("discretization param unset");
        ros::shutdown();
        return 0;
    }

    double max_velocity;
    if ( !private_nh.getParam("max_vel", max_velocity) ) {
        ROS_ERROR("Maximum velocity param unset");
        ros::shutdown();
        return 0;
    }

    double max_acceleration;
    if ( !private_nh.getParam("max_acc", max_acceleration) ) {
        ROS_ERROR("Maximum acceleration param unset");
        ros::shutdown();
        return 0;
    }

    double kP;
    if ( !private_nh.getParam("kP", kP) ) {
        ROS_ERROR("kP constant param unset");
        ros::shutdown();
        return 0;
    }

    double kD;
    if ( !private_nh.getParam("kD", kD) ) {
        ROS_ERROR("kD constant param unset");
        ros::shutdown();
        return 0;
    }

    PathFollower path_follower(nh, discretization, max_velocity, max_acceleration, kP, kD);

    while (ros::ok()) {
        try {
            path_follower.Update();
        }
        catch (tf2::TransformException e) {
            ROS_ERROR("Failed to lookup transform");
        }
        ros::spinOnce();
        rate.sleep();
    }
}
