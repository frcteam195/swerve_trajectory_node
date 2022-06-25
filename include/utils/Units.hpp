#pragma once

#include "CKMathConstants.hpp"

namespace ck
{
    namespace math
    {
        inline double deg2rad(double deg)
        {
            return deg * PI / 180.0;
        }

        inline float deg2radf(float deg)
        {
            return deg * PI_F / 180.0f;
        }

        inline double rad2deg(double rad)
        {
            return rad * 180.0 / PI;
        }

        inline float rad2degf(float rad)
        {
            return rad * 180.0f / PI_F;
        }

        inline double meters_to_inches(double meters)
        {
            return meters / 0.0254;
        }

        inline double inches_to_meters(double inches)
        {
            return inches * 0.0254;
        }

        inline double rpm_to_rads_per_sec(double rpm)
        {
            return rpm * 2.0 * PI / 60.0;
        }

        inline double rads_per_sec_to_rpm(double rads_per_sec)
        {
            return rads_per_sec * 60.0 / (2.0 * PI);
        }
    } // namespace math
} // namespace ck
