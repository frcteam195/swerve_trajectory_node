#pragma once

#include "geometry/Pose2d.hpp"
#include "geometry/Rotation2d.hpp"
#include "geometry/Twist2d.hpp"

namespace ck
{
    namespace math
    {
        /*
        ==========================================================================
 
  	        Provides forward and inverse kinematics equations for the robot 
            modeling the wheelbase as a differential drive (with a corrective 
            factor to account for skidding).
 
        ==========================================================================
        */
        class Kinematics 
        {
            public:
                // Forward kinematics using only encoders, rotation is implicit (less accurate than below, but useful for predicting motion).
                static geometry::Twist2d forwardKinematics(double left_wheel_delta, double right_wheel_delta);
                static geometry::Twist2d forwardKinematics(double left_wheel_delta, double right_wheel_delta, double delta_rotation_rads);
                static geometry::Twist2d forwardKinematics(geometry::Rotation2d previous_heading, double left_wheel_delta, double right_wheel_delta, geometry::Rotation2d current_heading);

                // For convenience, integrate forward kinematics with a Twist2d and previous rotation.
                static geometry::Pose2d integrateForwardKinematics(geometry::Pose2d current_pose, geometry::Twist2d forward_kinematics);
        };

    } // namespace math

} // namespace ck