#pragma once

#include "ck_utilities/team254_geometry/Pose2dWithCurvature.hpp"
#include "ck_utilities/team254_swerve/SwerveDriveKinematics.hpp"
#include "ck_utilities/planners/DriveMotionPlanner.hpp"
#include "ck_utilities/trajectory/timing/TimingConstraint.hpp"
#include "ck_utilities/trajectory/Trajectory.hpp"
#include "ck_utilities/trajectory/TimedView.hpp"
#include "ck_utilities/geometry/geometry_ros_helpers.hpp"
#include "ros/ros.h"


#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <string>
#include <vector>

constexpr double FIELD_WIDTH_INCHES = 315.5;
constexpr double FIELD_LENGTH_INCHES = 651.25;

struct TrajectorySet
{
    ck::trajectory::Trajectory<ck::trajectory::timing::TimedState<ck::team254_geometry::Pose2dWithCurvature>, ck::trajectory::timing::TimedState<ck::team254_geometry::Rotation2d>> red_trajectory;
    nav_msgs::Path red_path;
    ck::trajectory::Trajectory<ck::trajectory::timing::TimedState<ck::team254_geometry::Pose2dWithCurvature>, ck::trajectory::timing::TimedState<ck::team254_geometry::Rotation2d>> blue_trajectory;
    nav_msgs::Path blue_path;
};

struct PathPoint
{
    ck::team254_geometry::Pose2d waypoint;
    ck::team254_geometry::Rotation2d heading;
};

struct PathStruct
{
    std::vector<ck::team254_geometry::Pose2d> waypoints;
    std::vector<ck::team254_geometry::Rotation2d> headings;
};

struct PathSet
{
    PathStruct red;
    PathStruct blue;
    double max_velocity_in_per_sec;
    double max_accel_in_per_sec;
    double max_decel_in_per_sec;
    bool accel_smoothing;
};

std::vector<PathStruct> mirror_paths(std::vector<PathStruct> paths);
PathPoint mirror_point(PathPoint point);

void debug_trajectory(Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory);
void debug_trajectory_iterator(Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory, double timestep=0.1);
