#pragma once

#include "State.hpp"

namespace ck
{
    namespace geometry
    {
        class Translation2d;

        template <class S>
        class ITranslation2d : public State<S>
        {
        public:
            virtual Translation2d getTranslation() const = 0;
        };
    } // namespace geometry
} // namespace ck