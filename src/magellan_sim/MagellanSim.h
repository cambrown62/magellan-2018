#ifndef MAGELLANSIM_H_
#define MAGELLANSIM_H_

#include <ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

class MagellanSim {
public:
    MagellanSim(ros::NodeHandle& nh);
    void updateVelocity();
    void updateYaw();
    void update();

private:
    ros::NodeHandle& node_handle_;
    ros::Subscriber<std_msgs::Float64, MagellanSim> throttle_subscriber_;
    ros::Subscriber<std_msgs::Float64, MagellanSim> steering_subscriber_;
    ros::Publisher velocity_publisher_;
    ros::Publisher turning_radius_publisher_;
    std_msgs::Float64 turning_radius_msg_;
    geometry_msgs::TwistWithCovarianceStamped twist_msgs_;
    double commanded_throttle_;
    double commanded_steering_angle_;
    double velocity_;
    double steering_angle_;
    void updateThrottle(const std_msgs::Float64& cmd_velocity);
    void updateSteering(const std_msgs::Float64& cmd_radius);
};

#endif
