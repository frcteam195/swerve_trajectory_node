#pragma once

#include <cmath>
#include <vector>
#include "utils/CKMath.hpp"
#include "geometry/Geometry.hpp"

namespace ck
{
    namespace spline
    {
        class Spline
        {
        public:
            virtual ck::geometry::Translation2d getPoint(double t) = 0;
            virtual ck::geometry::Rotation2d getHeading(double t) = 0;
            virtual double getCurvature(double t) = 0;
            virtual double getDCurvature(double t) = 0;
            virtual double getVelocity(double t) = 0;
            virtual ck::geometry::Pose2d getPose2d(double t)
            {
                return ck::geometry::Pose2d(getPoint(t), getHeading(t));
            }
            virtual ck::geometry::Pose2dWithCurvature getPose2dWithCurvature(double t)
            {
                return ck::geometry::Pose2dWithCurvature(getPose2d(t), getCurvature(t), getDCurvature(t) / getVelocity(t));
            }
        };
    } // namespace physics
} // namespace ck