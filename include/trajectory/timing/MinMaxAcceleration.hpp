#pragma once

#include "ck_utilities/CKMath.hpp"

namespace ck
{
    namespace trajectory
    {
        namespace timing
        {
            class MinMaxAcceleration
            {
            public:
                double min_acceleration_;
                double max_acceleration_;
                static MinMaxAcceleration *kNoLimits;
                MinMaxAcceleration() : min_acceleration_(ck::math::NEG_INF), max_acceleration_(ck::math::POS_INF) {}
                MinMaxAcceleration(double min_acceleration, double max_acceleration) : min_acceleration_(min_acceleration), max_acceleration_(max_acceleration) {}
                double min_acceleration() { return min_acceleration_; }
                double max_acceleration() { return max_acceleration_; }
                bool valid() { return min_acceleration_ <= max_acceleration_; }
            };

            // MinMaxAcceleration::kNoLimits = new MinMaxAcceleration();
        } // namespace timing
    }     // namespace trajectory
} // namespace ck