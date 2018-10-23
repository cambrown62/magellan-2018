#ifndef GPS_ODOM_H
#define GPS_ODOM_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

class GPSOdom {
    public:
        GPSOdom(ros::NodeHandle& nh);

        void UpdateImu(const sensor_msgs::Imu::ConstPtr& imu);
        void UpdateFix(const sensor_msgs::NavSatFix::ConstPtr& fix);
        void UpdateVel(const geometry_msgs::TwistStamped::ConstPtr& twist);

        void Update();
    private:
        ros::Subscriber gps_subscriber_;
        ros::Subscriber imu_subscriber_;
        ros::Subscriber vel_subscriber_;

        ros::Publisher odom_publisher_;
        tf2_ros::TransformBroadcaster transform_broadcaster_;

        nav_msgs::Odometry odom_msg_;

        geometry_msgs::TransformStamped odom_transform_;
        geometry_msgs::TransformStamped utm_transform_;

        std::string base_link_frame_;
        std::string odom_frame_;
        std::string utm_frame_;

        bool origin_set_;
        double origin_x_, origin_y_;
};

#endif

