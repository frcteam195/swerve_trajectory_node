#pragma once

#include "State.hpp"

namespace ck
{
    namespace geometry
    {
        class Rotation2d;

        template <class S>
        class IRotation2d : public State<S>
        {
        public:
            virtual Rotation2d getRotation() const = 0;
        };
    } // namespace geometry
} // namespace ck