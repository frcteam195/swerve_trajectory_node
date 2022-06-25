#pragma once

#include <type_traits>
#include "geometry/State.hpp"
#include "TrajectorySamplePoint.hpp"

namespace ck
{
    namespace trajectory
    {
        template <class S>
        class Trajectory;

        template <class S>
        class TrajectoryView
        {
            static_assert(std::is_base_of<ck::geometry::State<S>, S>::value, "S must inherit from State");

        public:
            virtual ~TrajectoryView(){}
            virtual TrajectorySamplePoint<S> sample(double interpolant) = 0;
            virtual double first_interpolant() const = 0;
            virtual double last_interpolant() const = 0;
            virtual Trajectory<S> trajectory() = 0;
        };
    } // namespace trajectory
} // namespace ck
