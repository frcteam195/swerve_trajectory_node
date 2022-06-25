#pragma once

#include "utils/interpolables/Interpolable.hpp"

namespace ck
{
    namespace geometry
    {
        template <class S>
        class State : public ck::math::Interpolable<S>
        {
        public:
           virtual double distance(const S &other) const = 0;
           virtual bool equals(const S &other) = 0;
        };
    }
}