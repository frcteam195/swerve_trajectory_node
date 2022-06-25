#pragma once

#include <type_traits>
#include "utils/CKMath.hpp"
#include "geometry/State.hpp"
#include "TrajectoryView.hpp"
#include "TrajectorySamplePoint.hpp"

namespace ck
{
    namespace trajectory
    {
        template <class S>
        class TrajectoryIterator
        {
            static_assert(std::is_base_of<ck::geometry::State<S>, S>::value, "S must inherit from State");

        protected:
            TrajectoryView<S> *view_;
            double progress_;
            TrajectorySamplePoint<S> *current_sample_;

        public:

            TrajectoryIterator(TrajectoryView<S> *view)
                {
                    view_ = view;
                    progress_ = view->first_interpolant();
                    current_sample_ = new TrajectorySamplePoint<S>( view->sample(view->first_interpolant()) );
                }

            bool isDone()
                {
                    return getRemainingProgress() == 0.0;
                }

            double getProgress()
                {
                    return progress_;
                }

            double getRemainingProgress()
                {
                    return ck::math::max(0.0, view_->last_interpolant() - progress_);
                }

            TrajectorySamplePoint<S> getSample()
                {
                    return *current_sample_;
                }

            S getState()
                {
                    return (current_sample_->state());
                }

            TrajectorySamplePoint<S> advance(double additional_progress)
                {
                    progress_ = ck::math::max(view_->first_interpolant(), ck::math::min(view_->last_interpolant(), progress_ + additional_progress));
                    *current_sample_ = view_->sample(progress_);
                    return *current_sample_;
                }

            TrajectorySamplePoint<S> preview(double additional_progress)
                {
                    double progress = ck::math::max(view_->first_interpolant(), ck::math::min(view_->last_interpolant(), progress_ + additional_progress));
                    return view_->sample(progress);
                }
            Trajectory<S> trajectory() { return view_->trajectory(); }
        };
    } // namespace trajectory
} // namespace ck
