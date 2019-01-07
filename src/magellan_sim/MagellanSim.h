#ifndef MAGELLANSIM_H_
#define MAGELLANSIM_H_

#include <ros.h>
#include <time.h>
#include <duration.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

class MagellanSim {
public:
    MagellanSim(ros::NodeHandle& nh);
    void UpdateVelocity();
    void UpdateYaw();
    void Update();

private:
    ros::Time time_;
    ros::NodeHandle& node_handle_;
    ros::Subscriber throttle_subscriber_;
    ros::Subscriber steering_subscriber_;
    ros::Publisher velocity_publisher_;
    ros::Publisher turning_radius_publisher_;
    double commanded_velocity_;
    double commanded_radius_;
    double velocity_;
    double steering_angle_;

    void UpdateThrottle(const std_msgs::Float64& cmd_velocity);
    void UpdateSteering(const std_msgs::Float64& cmd_radius);
};

#endif
