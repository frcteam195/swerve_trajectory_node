#pragma once

#include <vector>
#include "geometry/State.hpp"
#include "TrajectoryView.hpp"
#include "TrajectorySamplePoint.hpp"
#include "timing/TimedState.hpp"
#include "utils/CKMath.hpp"

namespace ck
{
    namespace trajectory
    {
        template <class S>
        class TimedView : public TrajectoryView<ck::trajectory::timing::TimedState<S>>
        {
            static_assert(std::is_base_of<ck::geometry::State<S>, S>::value, "S must inherit from State<S>");

        protected:
            Trajectory<ck::trajectory::timing::TimedState<S>> *trajectory_;
            double start_t_;
            double end_t_;

        public:
            TimedView(Trajectory<ck::trajectory::timing::TimedState<S>> &traj)
                : trajectory_(&traj),
                  start_t_(traj.getState(0).t()),
                  end_t_(traj.getState(traj.length() - 1).t())
            {
                
            }

            double first_interpolant() const override { return start_t_; }
            double last_interpolant() const override { return end_t_; }

            /*
            TrajectorySamplePoint<ck::trajectory::timing::TimedState<S>> sample(double t) override
            {
                return TrajectorySamplePoint<ck::trajectory::timing::TimedState<S>>(
                    trajectory_->getPoint(trajectory_->length() - 1)
                    );
            }
            */

            TrajectorySamplePoint<ck::trajectory::timing::TimedState<S>> sample(double t) override
            {
                if (t >= end_t_)
                {
                    return TrajectorySamplePoint<ck::trajectory::timing::TimedState<S>>(trajectory_->getPoint(trajectory_->length() - 1));
                }
                if (t <= start_t_)
                {
                    return TrajectorySamplePoint<ck::trajectory::timing::TimedState<S>>(trajectory_->getPoint(0));
                }
                for (int i = 1; i < trajectory_->length(); ++i)
                {
                    TrajectoryPoint<ck::trajectory::timing::TimedState<S>> s = trajectory_->getPoint(i);
                    if (s.state_.t() >= t)
                    {
                        TrajectoryPoint<ck::trajectory::timing::TimedState<S>> prev_s = trajectory_->getPoint(i - 1);
                        if (ck::math::epsilonEquals(s.state_.t(), prev_s.state_.t()))
                        {
                            return TrajectorySamplePoint<ck::trajectory::timing::TimedState<S>>(s);
                        }
                        return TrajectorySamplePoint<ck::trajectory::timing::TimedState<S>>(prev_s.state_.interpolate(s.state_, (t - prev_s.state_.t()) / (s.state_.t() - prev_s.state_.t())), i - 1, i);
                    }
                }
                throw;
                // throw new RuntimeException();
            }

            Trajectory<ck::trajectory::timing::TimedState<S>> trajectory() override { return *trajectory_; }
        };
    } // namespace trajectory
} // namespace ck
