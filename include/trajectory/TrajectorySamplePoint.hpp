#pragma once

#include <type_traits>
#include "geometry/State.hpp"
#include "TrajectoryPoint.hpp"

namespace ck
{
    namespace trajectory
    {
        template <class S>
        class TrajectorySamplePoint
        {
        public:
            static_assert(std::is_base_of<ck::geometry::State<S>, S>::value, "S must inherit from State<S>");
            S state_;
            int index_floor_;
            int index_ceil_;


            int index_floor(){ return index_floor_; }
            int index_ceil(){ return index_ceil_; }
            S state(){ return state_; }

            TrajectorySamplePoint(const TrajectoryPoint<S> &point)
                : state_(point.state_),
                  index_floor_(point.index_),
                  index_ceil_(point.index_)
            {
                
            }

            TrajectorySamplePoint(S state, int index_floor, int index_ceil)
                : state_(state),
                  index_floor_(index_floor),
                  index_ceil_(index_ceil)
            {
            }

        };
    } // namespace trajectory
} // namespace ck
