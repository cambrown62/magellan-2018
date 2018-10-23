#include "GPSOdom.h"
#include "Conversions.h"

GPSOdom::GPSOdom(ros::NodeHandle& nh) :
        origin_set_(false),
        gps_subscriber_(nh.subscribe<sensor_msgs::NavSatFix>("fix", 10, &GPSOdom::UpdateFix, this)),
        imu_subscriber_(nh.subscribe<sensor_msgs::Imu>("imu", 10, &GPSOdom::UpdateImu, this)),
        vel_subscriber_(nh.subscribe<geometry_msgs::TwistStamped>("vel", 10, &GPSOdom::UpdateVel, this)),
        odom_publisher_(nh.advertise<nav_msgs::Odometry>("odometry/gps/raw", 10)),
        odom_frame_("odom"),
        base_link_frame_("boat"),
        utm_frame_("utm") {

    nh.param<std::string>("odom_frame", odom_frame, "odom");
    nh.param<std::string>("boat_frame", base_link_frame, "boat");
    nh.param<std::string>("utm_frame", utm_frame, "utm");

    odom_transform_.header.frame_id = odom_frame;
    odom_transform_.child_frame_id = base_link_frame;

    utm_transform_.header.frame_id = utm_frame;
    utm_transform_.child_frame_id = odom_frame;

    geometry_msgs::Quaternion zero_quat_;
    zeroQuat.x = zeroQuat.y = zeroQuat.z = 0;
    zeroQuat.w = 1;

    odomTransform.transform.rotation = zeroQuat;
    utmTransform.transform.rotation = zeroQuat;
}

void GPSOdom::UpdateFix(const sensor_msgs::NavSatFix::ConstPtr& fix) {
    if ( std::isnan(fix->latitude) || std::isnan(fix->longitude) ) {
        ROS_WARN("GPSOdom has no fix");
        return;
    }

    double northing, easting;
    std::string zone;
    gps_common::LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

    if ( !originSet ) {
        originX = easting;
        originY = northing;
        originSet = true;
        ROS_WARN("GPSOdom set origin - received first GPS message");
    }

    odom_msg.pose.pose.position.x = easting - originX;
    odom_msg.pose.pose.position.y = northing - originY;

    geometry_msgs::Vector3 position;
    position.x = odom_msg.pose.pose.position.x;
    position.y = odom_msg.pose.pose.position.y;
    position.z = 0;
    odomTransform.transform.translation = position;

    odomTransform.header.stamp = ros::Time::now();
    transform_broadcaster.sendTransform(odomTransform);

    if ( originSet ) {
        geometry_msgs::Vector3 position;
        position.x = originX;
        position.y = originY;
        position.z = 0;
        utmTransform.transform.translation = position;
        utmTransform.header.stamp = ros::Time::now();
        transform_broadcaster.sendTransform(utmTransform);
    }
}

void GPSOdom::UpdateImu(const sensor_msgs::Imu::ConstPtr& imu) {
    odom_msg.pose.pose.orientation = imu->orientation;
    odomTransform.transform.rotation = imu->orientation;
}

void GPSOdom::UpdateVel(const geometry_msgs::TwistStamped::ConstPtr& twist) {
    odom_msg.twist.twist = twist->twist;
}

void GPSOdom::Update() {
    odomPub.publish(odom_msg);
}
