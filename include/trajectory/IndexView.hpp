#pragma once

#include "ck_utilities/CKMath.hpp"
#include "ck_utilities/geometry/State.hpp"

#include "TrajectorySamplePoint.hpp"
#include "TrajectoryView.hpp"

// #include <iostream>
#include <vector>

namespace ck
{
    namespace trajectory
    {
        template <class S>
        class IndexView : public TrajectoryView<S>
        {
            static_assert(std::is_base_of<ck::geometry::State<S>, S>::value, "S must inherit from State<S>");

        protected:
            Trajectory<S> *trajectory_;
            bool is_initialized;
        public:
            IndexView()
            {
                is_initialized = false;
            }

            void initialize(Trajectory<S>* traj)
            {
                trajectory_ = traj;
                is_initialized = true;
            }

            TrajectorySamplePoint<S> sample(double index) override { return trajectory_->getInterpolated(index); }
            double last_interpolant() const override { return ck::math::max(0, trajectory_->length() - 1); }
            double first_interpolant() const override { return 0.0; }
            Trajectory<S> trajectory() override { return *trajectory_; }
        };
    } // namespace trajectory
} // namespace ck
