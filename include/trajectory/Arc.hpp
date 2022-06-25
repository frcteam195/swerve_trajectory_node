#pragma once

#include <vector>
#include <cmath>
#include <limits>
#include "geometry/Geometry.hpp"
#include "utils/CKMath.hpp"

namespace ck
{
    namespace trajectory
    {
        template <class S>
        class PurePursuitController;

        template <class S>
        class Arc
        {
            static_assert(std::is_base_of<ck::geometry::ITranslation2d<S>, S>::value, "S must inherit from ITranslation2d<S>");

        public:
            ck::geometry::Translation2d center;
            double radius;
            double length;

            Arc(const ck::geometry::Pose2d &pose, const S &point) : center(findCenter(pose, point)), radius(ck::geometry::Translation2d(center, point.getTranslation()).norm() * PurePursuitController::getDirection(pose, point)), length(findLength(pose, point, center, radius)) {}

        protected:
            ck::geometry::Translation2d findCenter(ck::geometry::Pose2d &pose, S &point)
            {
                ck::geometry::Translation2d poseToPointHalfway = pose.getTranslation().interpolate(point.getTranslation(), 0.5);
                ck::geometry::Rotation2d normal = pose.getTranslation().inverse().translateBy(poseToPointHalfway).direction().normal();
                ck::geometry::Pose2d perpendicularBisector(poseToPointHalfway, normal);
                ck::geometry::Pose2d normalFromPose(pose.getTranslation(), pose.getRotation().normal());
                if (normalFromPose.isColinear(perpendicularBisector.normal()))
                {
                    // Special case: center is poseToPointHalfway.
                    return poseToPointHalfway;
                }
                return normalFromPose.intersection(perpendicularBisector);
            }

            double findLength(ck::geometry::Pose2d &pose, S &point, ck::geometry::Translation2d &center, double radius)
            {
                if (radius < std::numeric_limits<double>::max())
                {
                    ck::geometry::Translation2d centerToPoint(center, point.getTranslation());
                    ck::geometry::Translation2d centerToPose(center, pose.getTranslation());
                    // If the point is behind pose, we want the opposite of this angle. To determine if the point is behind,
                    // check the sign of the cross-product between the normal vector and the vector from pose to point.
                    bool behind = ck::math::signum(
                                      ck::geometry::Translation2d::cross(pose.getRotation().normal().toTranslation(), ck::geometry::Translation2d(pose.getTranslation(), point.getTranslation()))) > 0.0;
                    ck::geometry::Rotation2d angle = ck::geometry::Translation2d::getAngle(centerToPose, centerToPoint);
                    return radius * (behind ? 2.0 * Math.PI - std::fabs(angle.getRadians()) : std::fabs(angle.getRadians()));
                }
                else
                {
                    return Translation2d(pose.getTranslation(), point.getTranslation()).norm();
                }
            }
        };
    } // namespace trajectory
} // namespace ck