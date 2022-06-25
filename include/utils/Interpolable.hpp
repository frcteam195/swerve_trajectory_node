#pragma once

namespace ck
{
    namespace math
    {
        template <class T>
        class Interpolable
        {
            virtual T interpolate(const T &other, double x) const = 0;
        };
    }
}