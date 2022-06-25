#pragma once

#include "Interpolable.hpp"
#include "InverseInterpolable.hpp"

namespace ck
{
    namespace math
    {
        class InterpolatingDouble : public Interpolable<InterpolatingDouble>, public InverseInterpolable<InterpolatingDouble>
        {
        public:
            double value;
            InterpolatingDouble(double val) : value(val) {}
            InterpolatingDouble interpolate(InterpolatingDouble &other, double x) override
            {
                double dydx = other.value - value;
                double searchY = dydx * x + value;
                return InterpolatingDouble(searchY);
            }
            double inverseInterpolate(const InterpolatingDouble &upper, const InterpolatingDouble &query) override
            {
                double upper_to_lower = upper.value - value;
                if (upper_to_lower <= 0)
                {
                    return 0;
                }
                double query_to_lower = query.value - value;
                if (query_to_lower <= 0)
                {
                    return 0;
                }
                return query_to_lower / upper_to_lower;
            }
            int compareTo(InterpolatingDouble &other)
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