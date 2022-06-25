#include "Constants.hpp"
#include "utils/Kinematics.hpp"

namespace ck 
{
    namespace math
    {
        
        geometry::Twist2d Kinematics::forwardKinematics(double left_wheel_delta, double right_wheel_delta)
        {
            double delta_rotation = (right_wheel_delta - left_wheel_delta) / (K_DRIVE_WHEEL_TRACK_WIDTH_INCHES * K_TRACK_SCRUB_FACTOR);
            return forwardKinematics(left_wheel_delta, right_wheel_delta, delta_rotation);
        }
        
        geometry::Twist2d Kinematics::forwardKinematics(double left_wheel_delta, double right_wheel_delta, double delta_rotation_rads)
        {
            const double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
            const double dy = 0.0;
            return geometry::Twist2d(dx, dy, delta_rotation_rads);
        }
        
        geometry::Twist2d Kinematics::forwardKinematics(geometry::Rotation2d previous_heading, double left_wheel_delta, double right_wheel_delta, geometry::Rotation2d current_heading)
        {
            const double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
            const double dy = 0.0;
            return geometry::Twist2d(dx, dy, previous_heading.inverse().rotateBy(current_heading).getRadians());
        }

        geometry::Pose2d Kinematics::integrateForwardKinematics(geometry::Pose2d current_pose, geometry::Twist2d forward_kinematics)
        {
            return current_pose.transformBy(geometry::Pose2d::exp(forward_kinematics));
        }

    } // namespace math

} // namespace ck