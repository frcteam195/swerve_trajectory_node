#pragma once

#include <type_traits>
#include "geometry/State.hpp"

namespace ck
{
    namespace trajectory
    {
        template <class S>
        struct TrajectoryPoint
        {
            static_assert(std::is_base_of<ck::geometry::State<S>, S>::value, "S must inherit from State<S>");
            S state_;
            int index_;
        };
    }
}
