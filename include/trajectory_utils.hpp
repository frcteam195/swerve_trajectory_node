#pragma once

#include "ck_utilities/team254_geometry/Pose2dWithCurvature.hpp"
#include "ck_utilities/team254_swerve/SwerveDriveKinematics.hpp"
#include "ck_utilities/planners/DriveMotionPlanner.hpp"
#include "ck_utilities/trajectory/timing/TimingConstraint.hpp"
#include "ck_utilities/trajectory/Trajectory.hpp"
#include "ck_utilities/trajectory/TimedView.hpp"
#include "ck_utilities/geometry/geometry_ros_helpers.hpp"

void debug_trajectory(Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory);
void debug_trajectory_iterator(Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory, double timestep=0.1);
