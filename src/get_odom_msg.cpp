#include "get_odom_msg.hpp"
#include <iostream>

using namespace nav_msgs;

Odometry get_odom_msg(double x_inches, double y_inches, double heading_degrees)
{
    std::cout << "RESET: " << x_inches << "," << y_inches << "," << heading_degrees << std::endl; 

    Odometry odom = Odometry();

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    geometry::Translation position;
    position.x(ck::math::inches_to_meters(x_inches));
    position.y(ck::math::inches_to_meters(y_inches));
    position.z(0.0);

    geometry::Rotation orientation;
    orientation.roll(0.0);
    orientation.pitch(0.0);
    orientation.yaw(ck::math::deg2rad(heading_degrees));

    odom.pose.pose.position = geometry::to_msg_point(position);
    odom.pose.pose.orientation = geometry::to_msg_quat(orientation);

    geometry::Translation linear;
    geometry::Rotation angular;

    odom.twist.twist.linear = geometry::to_msg(linear);
    odom.twist.twist.angular = geometry::to_msg(angular);

    geometry::Covariance pose_cov;
    pose_cov.x_var(0.001);
    pose_cov.y_var(0.001);
    pose_cov.z_var(0.001);
    pose_cov.roll_var(0.001);
    pose_cov.pitch_var(0.001);
    pose_cov.yaw_var(0.00001);

    geometry::Covariance twist_cov;
    twist_cov.x_var(0.001);
    twist_cov.y_var(0.001);
    twist_cov.z_var(0.001);
    twist_cov.roll_var(0.001);
    twist_cov.pitch_var(0.001);
    twist_cov.yaw_var(0.001);

    odom.pose.covariance = geometry::to_msg(pose_cov);
    odom.twist.covariance = geometry::to_msg(twist_cov);

    return odom;
}
