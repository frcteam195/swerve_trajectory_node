#include "geometry/Twist2d.hpp"

namespace ck
{
    namespace geometry
    {
        const Twist2d &Twist2d::identity()
        {
            static Twist2d kIdentity;
            return kIdentity;
        }

        Twist2d::Twist2d() : dx(0), dy(0), dtheta(0) {}
        Twist2d::Twist2d(double dx, double dy, double dtheta) : dx(dx), dy(dy), dtheta(dtheta) {}

        std::ostream &operator<<(std::ostream &os, const Twist2d &tw2d)
        {
            os << std::setprecision(3) << "(dx: " << tw2d.dx << ", dy: " << tw2d.dy << ", dtheta: " << tw2d.dtheta << ")";
            return os;
        }

        Twist2d Twist2d::scaled(double scale) const
        {
            return Twist2d(dx * scale, dy * scale, dtheta * scale);
        }

        double Twist2d::norm() const
        {
            // Common case of dy == 0
            if (dy == 0.0)
            {
                return std::abs(dx);
            }
            return std::hypot(dx, dy);
        }

        double Twist2d::curvature() const
        {
            //TODO: Determine if this should be an || here, as if norm is less than kEpsilon, could be divide by 0
            if (std::abs(dtheta) < ck::math::kEpsilon && norm() < ck::math::kEpsilon)
            {
                return 0.0;
            }
            return dtheta / norm();
        }

    } // namespace geometry
} // namespace ck