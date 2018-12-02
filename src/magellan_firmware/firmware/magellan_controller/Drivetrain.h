#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "config.h"
#include <ros.h>
#include <std_msgs/Float64.h>
#include "PWM.h"

class Drivetrain {
public:
    Drivetrain(ros::NodeHandle& nh);

    void SetThrottlePercent(double percent);
    void SetSteeringPercent(double percent);
    void SetSteeringAngle(double angle);
    bool DirectionIsForward();

    double GetSteeringAngleForPercent(double percent);
    double GetPercentForSteeringAngle(double angle);
    double GetTurningRadius(double percent);

    void Update();
private:
    double last_commanded_throttle_;
    double last_commanded_steering_;
    ros::NodeHandle& nh_;
    PWM throttle_pwm_;
    PWM steering_pwm_;

    std_msgs::Float64 turning_radius_msg_;
    ros::Publisher turning_radius_publisher_;
};

#endif
