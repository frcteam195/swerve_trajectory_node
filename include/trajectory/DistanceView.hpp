#pragma once

#include <vector>
#include <cmath>
#include <type_traits>
#include "geometry/State.hpp"
#include "utils/CKMath.hpp"
#include "TrajectoryView.hpp"

namespace ck
{
    namespace trajectory
    {
        template <class S>
        class DistanceView : public TrajectoryView<S>
        {
            static_assert(std::is_base_of<ck::geometry::State<S>, S>::value, "S must inherit from State<S>");

        protected:
            Trajectory<S> *trajectory_;
            std::vector<double> distances_;

        public:
            DistanceView(Trajectory<S> &trajectory) : trajectory_(&trajectory), distances_(trajectory.length())
            {
                distances_[0] = 0.0;
                for (int i = 1; i < trajectory_->length(); ++i)
                {
                    distances_[i] = distances_[i - 1] + trajectory_->getState(i - 1).distance(trajectory_->getState(i));
                }
            }

            TrajectorySamplePoint<S> sample(double distance) override
            {
                if (distance >= last_interpolant())
                    return TrajectorySamplePoint<S>(trajectory_->getPoint(trajectory_->length() - 1));
                if (distance <= 0.0)
                    return TrajectorySamplePoint<S>(trajectory_->getPoint(0));
                for (size_t i = 1; i < distances_.size(); ++i)
                {
                    TrajectoryPoint<S> s = trajectory_->getPoint(i);
                    if (distances_[i] >= distance)
                    {
                        TrajectoryPoint<S> prev_s = trajectory_->getPoint(i - 1);
                        if (ck::math::epsilonEquals(distances_[i], distances_[i - 1]))
                        {
                            return TrajectorySamplePoint<S>(s);
                        }
                        else
                        {
                            return TrajectorySamplePoint<S>(prev_s.state_.interpolate(s.state_, (distance - distances_[i - 1]) / (distances_[i] - distances_[i - 1])), i - 1, i);
                        }
                    }
                }
                throw;
            }

            double last_interpolant() const override
            {
                return distances_[distances_.size() - 1];
            }

            double first_interpolant() const override
            {
                return 0.0;
            }

            Trajectory<S> trajectory() override
            {
                return *trajectory_;
            }
        };
    } // namespace trajectory
} // namespace ck