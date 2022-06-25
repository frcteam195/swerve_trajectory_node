#pragma once

#include <type_traits>
#include <vector>
#include <cmath>
#include <limits>
#include "geometry/State.hpp"
#include "IndexView.hpp"

namespace ck
{
    namespace trajectory
    {
        template <class S>
        class Trajectory
        {
            static_assert(std::is_base_of<ck::geometry::State<S>, S>::value, "S must inherit from State<S>");

        protected:
            std::vector<TrajectoryPoint<S>> points_;
            IndexView<S> index_view_;

        public:
            Trajectory()
                : points_()
            {
                index_view_.initialize(this);
            }

            Trajectory(std::vector<S> &states)
                : points_()
            {
                index_view_.initialize(this);
                for (int i = 0; i < (int)states.size(); ++i)
                {
                    points_.push_back(TrajectoryPoint<S>{states[i], i});
                }
            }

            Trajectory( const Trajectory& in ) : Trajectory() {
                points_ = in.points_;
            }

            Trajectory& operator=( const Trajectory& in ){
                points_ = in.points_;
                return *this;
            }

            bool isEmpty() {return points_.empty();}
            int length() { return points_.size(); }
            TrajectoryPoint<S> getPoint(int index) { return points_[index]; }
            S getState(int index) { return points_[index].state_; }
            S getFirstState() { return getState(0); }
            S getLastState() { return getState(length() - 1); }

            TrajectorySamplePoint<S> getInterpolated(double index)
            {
                if (isEmpty())
                {
                    //TODO: Check if this is okay instead of returning null
                    throw;
                }
                else if (index <= 0.0)
                {
                    return TrajectorySamplePoint<S>(points_[0]);
                }
                else if (index >= length() - 1)
                {
                    return TrajectorySamplePoint<S>(points_[length() - 1]);
                }
                int i = (int)std::floor(index);
                double frac = index - i;
                if (frac <= std::numeric_limits<double>::min())
                {
                    return TrajectorySamplePoint<S>(points_[i]);
                }
                else if (frac >= 1.0 - std::numeric_limits<double>::min())
                {
                    return TrajectorySamplePoint<S>(points_[i + 1]);
                }
                else
                {
                    return TrajectorySamplePoint<S> { getState(i).interpolate(getState(i + 1), frac), i, i + 1 };
                }
            }
            IndexView<S>& getIndexView() { return index_view_; }
        };
    } // namespace trajectory
} // namespace ck
