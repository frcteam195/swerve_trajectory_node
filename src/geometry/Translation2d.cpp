#include "geometry/Translation2d.hpp"

namespace ck
{
    namespace geometry
    {
        const Translation2d &Translation2d::identity()
        {
            static Translation2d kIdentity;
            return kIdentity;
        }
        Translation2d::Translation2d() : x_(0), y_(0) {}
        Translation2d::Translation2d(double x, double y) : x_(x), y_(y) {}
        Translation2d::Translation2d(const Translation2d &start, const Translation2d &end) : x_(end.x_ - start.x_), y_(end.y_ - start.y_) {}
        Translation2d::~Translation2d() {}

        Translation2d Translation2d::operator+(const Translation2d &obj) const
        {
            return Translation2d(x_+ obj.x_, y_ + obj.y_);
        }
        //Do not use. Instead, + by a negative translation
        Translation2d Translation2d::operator-(const Translation2d &obj) const
        {
            return Translation2d(x_ - obj.x_, y_ - obj.y_);
        }
        bool Translation2d::operator==(const Translation2d &obj) const
        {
            return distance(obj) < ck::math::kEpsilon;
        }

        std::ostream &operator<<(std::ostream &os, const Translation2d &t2d)
        {
            os << std::setprecision(3) << "(x: " << t2d.x_ << ", y: " << t2d.y_ << ")";
            return os;
        }

        double Translation2d::norm() const
        {
            return std::hypot(x_, y_);
        }

        double Translation2d::norm2() const
        {
            return ((x_ * x_) + (y_ * y_));
        }

        double Translation2d::x() const { return x_; }
        double Translation2d::y() const { return y_; }

        Translation2d Translation2d::translateBy(const Translation2d &other) const
        {
            return *this + other;
        }

        Translation2d Translation2d::rotateBy(const Rotation2d &rotation) const
        {
            return Translation2d(x_ * rotation.cos() - y_ * rotation.sin(), x_ * rotation.sin() + y_ * rotation.cos());
        }

        Rotation2d Translation2d::direction() const
        {
            return Rotation2d(x_, y_, true);
        }

        Translation2d Translation2d::inverse() const
        {
            return Translation2d(-x_, -y_);
        }

        Translation2d Translation2d::interpolate(const Translation2d &other, double interpFactor) const
        {
            if (interpFactor <= 0)
            {
                return Translation2d(*this);
            }
            else if (interpFactor >= 1)
            {
                return Translation2d(other);
            }
            return Translation2d(extrapolate(other, interpFactor));
        }

        Translation2d Translation2d::extrapolate(const Translation2d &other, double interpFactor) const
        {
            return Translation2d(interpFactor * (other.x_ - x_) + x_, interpFactor * (other.y_ - y_) + y_);
        }

        Translation2d Translation2d::scale(double s) const
        {
            return Translation2d(x_ * s, y_ * s);
        }

        bool Translation2d::epsilonEquals(const Translation2d &other, double epsilon) const
        {
            return ck::math::epsilonEquals(x_, other.x_, epsilon) && ck::math::epsilonEquals(y_, other.y_, epsilon);
        }

        double Translation2d::dot(const Translation2d &a, const Translation2d &b)
        {
            return a.x_ * b.x_ + a.y_ * b.y_;
        }

        Rotation2d Translation2d::getAngle(const Translation2d &a, const Translation2d &b)
        {
            double cos_angle = dot(a, b) / (a.norm() * b.norm());
            if (std::isnan(cos_angle))
            {
                return Rotation2d();
            }
            return Rotation2d::fromRadians(std::acos(ck::math::min(1.0, ck::math::max(cos_angle, -1.0))));
        }

        double Translation2d::cross(const Translation2d &a, const Translation2d &b)
        {
            return a.x_ * b.y_ - a.y_ * b.x_;
        }

        double Translation2d::distance(const Translation2d &other) const
        {
            return inverse().translateBy(other).norm();
        }

        Translation2d Translation2d::getTranslation() const {
            return *this;
        }

        bool Translation2d::equals(const Translation2d &other) {
            return *this == other;
        }

    } // namespace geometry
} // namespace ck
