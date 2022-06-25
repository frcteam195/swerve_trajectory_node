#pragma once

#include <limits>

namespace ck
{
    namespace math
    {
        constexpr double kEpsilon = 1e-12;
        constexpr double PI = 3.14159265358979323846;
        constexpr float PI_F = 3.14159265358979323846f;
        constexpr double POS_INF = std::numeric_limits<double>::infinity();
        constexpr double NEG_INF = -POS_INF;
        constexpr double POS_INF_F = std::numeric_limits<float>::infinity();
        constexpr double NEG_INF_F = -POS_INF_F;
    }
}