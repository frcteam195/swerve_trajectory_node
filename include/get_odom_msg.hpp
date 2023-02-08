#pragma once

#include "ros/ros.h"
#include "ck_utilities/CKMath.hpp"
#include "ck_utilities/geometry/geometry_ros_helpers.hpp"

#include "nav_msgs/Odometry.h"

nav_msgs::Odometry get_odom_msg(double x_inches, double y_inches, double heading_degrees);
