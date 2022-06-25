#pragma once

namespace ck
{
    namespace math
    {
        template <class T>
        class InverseInterpolable
        {
            virtual double inverseInterpolate(const T &upper, const T &query) = 0;
        };
    }
}