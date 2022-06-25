#pragma once

#include "CKMathConstants.hpp"
#include "Units.hpp"
#include <map>

namespace ck
{
    namespace math
    {
        template <typename T>
        inline T max(T a, T b)
        {
            return a > b ? a : b;
        }

        template <typename T>
        inline T min(T a, T b)
        {
            return a < b ? a : b;
        }

        template <typename T>
        inline int signum(T val)
        {
            return (T(0) < val) - (val < T(0));
        }

        template <typename T>
        inline bool epsilonEquals(T const &a, T const &b, T epsilon)
        {
            return (a - epsilon <= b) && (a + epsilon >= b);
        }

        template <typename T>
        inline bool epsilonEquals(T const &a, T const &b)
        {
            return epsilonEquals(a, b, kEpsilon);
        }

        template <typename T>
        inline T limit(T v, T maxMagnitude)
        {
            return limit(v, -maxMagnitude, maxMagnitude);
        }

        template <typename T>
        inline T limit(T v, T minVal, T maxVal)
        {
            return min(maxVal, max(minVal, v));
        }

        template <typename T>
        inline T interpolate(T a, T b, T x)
        {
            x = limit(x, 0.0, 1.0);
            return a + (b - a) * x;
        }

        template <typename K, typename V>
        inline V interpolate(const std::map<K, V> &data, K x)
        {
            typedef typename std::map<K, V>::const_iterator i_t;

            i_t i = data.upper_bound(x);
            if (i == data.end())
            {
                return (--i)->second;
            }
            if (i == data.begin())
            {
                return i->second;
            }
            i_t l = i;
            --l;

            const K delta = (x - l->first) / (i->first - l->first);
            return delta * i->second + (1 - delta) * l->second;
        }

        template <typename K, typename V>
        inline V interpolateGeometry2d(const std::map<K, V> &data, K x)
        {
            typedef typename std::map<K, V>::const_iterator i_t;

            i_t i = data.upper_bound(x);
            if (i == data.end())
            {
                return (--i)->second;
            }
            if (i == data.begin())
            {
                return i->second;
            }
            i_t l = i;
            --l;

            const K delta = (x - l->first) / (i->first - l->first);
            return i->second.interpolate(l->second, delta);
        }

    } // namespace math
} // namespace ck
