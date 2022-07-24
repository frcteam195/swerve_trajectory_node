#pragma once

#include "ck_utilities/geometry/Pose2dWithCurvature.hpp"
#include "MinMaxAcceleration.hpp"
#include "TimingConstraint.hpp"

#include <cmath>

namespace ck
{
    namespace trajectory
    {
        namespace timing
        {
            class CentripetalAccelerationConstraint : public TimingConstraint<ck::geometry::Pose2dWithCurvature>
            {
            public:
                double mMaxCentripetalAccel;
                CentripetalAccelerationConstraint(double max_centripetal_accel) : mMaxCentripetalAccel(max_centripetal_accel) {}
                double getMaxVelocity(const ck::geometry::Pose2dWithCurvature &state) const override
                {
                    return std::sqrt(std::fabs(mMaxCentripetalAccel / state.getCurvature()));
                }
                MinMaxAcceleration getMinMaxAcceleration(const ck::geometry::Pose2dWithCurvature &state, double velocity) const override
                {
                    (void)state;
                    (void)velocity;
                    // return MinMaxAcceleration::kNoLimits;
                    return MinMaxAcceleration();
                }
            };
        } // namespace timing
    }     // namespace trajectory
} // namespace ck