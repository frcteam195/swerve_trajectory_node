#pragma once

#include "Interpolable.hpp"
#include "InverseInterpolable.hpp"

namespace ck
{
    namespace math
    {
        class InterpolatingLong : public Interpolable<InterpolatingLong>, public InverseInterpolable<InterpolatingLong>
        {
        public:
            long value;
            InterpolatingLong(long val) : value(val) {}
            InterpolatingLong interpolate(InterpolatingLong &other, double x)
            {
                long dydx = other.value - value;
                double searchY = dydx * x + value;
                return InterpolatingLong((long)searchY);
            }
            double inverseInterpolate(InterpolatingLong &upper, InterpolatingLong &query)
            {
                long upper_to_lower = upper.value - value;
                if (upper_to_lower <= 0)
                {
                    return 0;
                }
                long query_to_lower = query.value - value;
                if (query_to_lower <= 0)
                {
                    return 0;
                }
                return query_to_lower / (double)upper_to_lower;
            }
            int compareTo(InterpolatingLong &other)
            {
                if (other.value < value)
                {
                    return 1;
                }
                else if (other.value > value)
                {
                    return -1;
                }
                else
                {
                    return 0;
                }
            }
        };
    } // namespace math
} // namespace ck