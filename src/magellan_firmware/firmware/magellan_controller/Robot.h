#ifndef ROBOT_H
#define ROBOT_H

#include "RobotState.h"
#include "TransmitterInterface.h"
#include "Rate.h"
#include "config.h"
#include <ros.h>
#include "HeartbeatLED.h"
#include "PWM.h"
#include <magellan_core/ActuatorCommand.h>
#include "IMU.h"
#include "EncoderPublisher.h"
#include "Drivetrain.h"

class Robot {
public:
    Robot(ros::NodeHandle& nh);

    void TeleopInit();
    void TeleopPeriodic();
    void AutonomousInit();
    void AutonomousPeriodic();
    void DisabledInit();
    void DisabledPeriodic();
    void AlwaysPeriodic();
    void UpdateActuators(const magellan_core::ActuatorCommand& cmd_actuators);
    void Update();
private:
    RobotState current_state_;
    ros::NodeHandle& nh_;
    TransmitterInterface transmitter_;
    HeartbeatLED heartbeat_;
    ros::Subscriber<magellan_core::ActuatorCommand, Robot> command_subscriber_;
    float throttle_percent_;
    float steering_angle_;
    IMU imu_;
    EncoderPublisher encoder_publisher_;
    Drivetrain drivetrain_;
};

#endif
