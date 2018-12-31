#include "MagellanSim.h"

MagellanSim::MagellanSim(ros::NodeHandle& nh) :
    node_handle_(nh),
    commanded_throttle_(0.0),
    throttle_subscriber_("/platform/cmd_velocity", &Robot::updateVelocity, this),
    steering_subscriber_("/platform/cmd_turning_radius", &Robot::updateSteering, this),
    commanded_steering_angle_(0.0),
    velocity_(0.0),
    steering_angle_(0.0) {
    
}
