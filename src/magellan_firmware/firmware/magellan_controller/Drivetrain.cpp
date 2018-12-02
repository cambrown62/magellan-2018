#include "Drivetrain.h"
#include "config.h"
#include <cmath>

Drivetrain::Drivetrain(ros::NodeHandle& nh) :
        last_commanded_throttle_(0),
        last_commanded_steering_(0),
        nh_(nh),
        throttle_pwm_(ESC_PWM),
        steering_pwm_(SERVO_PWM),
        turning_radius_msg_(),
        turning_radius_publisher_("/platform/turning_radius", &turning_radius_msg_) {
    throttle_pwm_.ConfigLowLimit(THROTTLE_MIN);
    steering_pwm_.ConfigOffset(STEERING_OFFSET);
    steering_pwm_.ConfigLowLimit(STEERING_MIN);

    nh.advertise(turning_radius_publisher_);
}

void Drivetrain::SetThrottlePercent(double percent) {
    throttle_pwm_.Set(percent);

    if ( fabs(percent) > 0.1 )
        last_commanded_throttle_ = percent;
}

void Drivetrain::SetSteeringPercent(double percent) {
    steering_pwm_.Set(percent);
    last_commanded_steering_ = percent;
}

void Drivetrain::SetSteeringAngle(double angle) {
    SetSteeringPercent(GetPercentForSteeringAngle(angle));
}

double Drivetrain::GetSteeringAngleForPercent(double percent) {
    return percent * kMaxTurningAngle;
}

double Drivetrain::GetPercentForSteeringAngle(double angle) {
    if ( std::abs(angle) > kMaxTurningAngle )
        return std::copysign(1, angle);
    return angle / kMaxTurningAngle;
}

// Returns radius of imaginary circle the car is driving along while turning
double Drivetrain::GetTurningRadius(double percent) {
    if ( percent == 0 )
        return 0;

    return kTrackLength * (1.0 / tan(GetSteeringAngleForPercent(percent)));
}

bool Drivetrain::DirectionIsForward() {
    return last_commanded_throttle_ >= 0;
}

void Drivetrain::Update() {
    turning_radius_msg_.data = GetTurningRadius(last_commanded_steering_);
    turning_radius_publisher_.publish(&turning_radius_msg_);
}
