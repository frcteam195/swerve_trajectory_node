#include "geometry/Pose2d.hpp"

namespace ck
{
    namespace geometry
    {
        const Pose2d &Pose2d::identity()
        {
            static Pose2d kIdentity;
            return kIdentity;
        }
        Pose2d::Pose2d() : translation(), rotation() {}
        Pose2d::Pose2d(double x, double y, const Rotation2d &rotation) : translation(x, y), rotation(rotation) {}
        Pose2d::Pose2d(const Translation2d &translation, const Rotation2d &rotation) : translation(translation), rotation(rotation) {}

        bool Pose2d::operator==(const Pose2d &obj) const
        {
            return epsilonEquals(obj, ck::math::kEpsilon);
        }

        std::ostream &operator<<(std::ostream &os, const Pose2d &p2d)
        {
            os << std::setprecision(3) << "(T: " << p2d.translation << ", R: " << p2d.rotation << ")";
            return os;
        }

        Pose2d Pose2d::fromTranslation(const Translation2d &translation)
        {
            return Pose2d(translation, Rotation2d());
        }

        Pose2d Pose2d::fromRotation(const Rotation2d &rotation)
        {
            return Pose2d(Translation2d(), rotation);
        }

        Pose2d Pose2d::exp(const Twist2d &delta)
        {
            double sin_theta = std::sin(delta.dtheta);
            double cos_theta = std::cos(delta.dtheta);
            double s, c;

            //TODO: check if replacing epsilon here is okay. Originally 1e-9, but using 1e-12
            if (std::abs(delta.dtheta) < (1e-9))
            {
                s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
                c = .5 * delta.dtheta;
            }
            else
            {
                s = sin_theta / delta.dtheta;
                c = (1.0 - cos_theta) / delta.dtheta;
            }
            return Pose2d(Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                          Rotation2d(cos_theta, sin_theta, false));
        }

        Twist2d Pose2d::log(const Pose2d &transform)
        {
            double dtheta = transform.getRotation().getRadians();
            double half_dtheta = 0.5 * dtheta;
            double cos_minus_one = transform.getRotation().cos() - 1.0;
            double halftheta_by_tan_of_halfdtheta;
            if (std::abs(cos_minus_one) < ck::math::kEpsilon)
            {
                halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
            }
            else
            {
                halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().sin()) / cos_minus_one;
            }
            Translation2d translation_part = transform.getTranslation().rotateBy(Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
            return Twist2d(translation_part.x(), translation_part.y(), dtheta);
        }

        Translation2d Pose2d::getTranslation() const
        {
            return translation;
        }

        Rotation2d Pose2d::getRotation() const
        {
            return rotation;
        }

        Pose2d Pose2d::transformBy(const Pose2d &other) const
        {
            return Pose2d(translation.translateBy(other.translation.rotateBy(rotation)), rotation.rotateBy(other.rotation));
        }

        Pose2d Pose2d::inverse() const
        {
            Rotation2d rotation_inverted = rotation.inverse();
            return Pose2d(translation.inverse().rotateBy(rotation_inverted), rotation_inverted);
        }

        Pose2d Pose2d::normal() const
        {
            return Pose2d(translation, rotation.normal());
        }

        Translation2d Pose2d::intersection(const Pose2d &other) const
        {
            Rotation2d other_rotation = other.getRotation();
            if (rotation.isParallel(other_rotation))
            {
                // Lines are parallel.
                return Translation2d(ck::math::POS_INF, ck::math::POS_INF);
            }
            if (std::abs(rotation.cos()) < std::abs(other_rotation.cos()))
            {
                return intersectionInternal(*this, other);
            }
            else
            {
                return intersectionInternal(other, *this);
            }
        }

        bool Pose2d::isColinear(const Pose2d &other) const
        {
            if (!getRotation().isParallel(other.getRotation()))
            {
                return false;
            }
            Twist2d twist = log(inverse().transformBy(other));
            return (ck::math::epsilonEquals(twist.dy, 0.0) && ck::math::epsilonEquals(twist.dtheta, 0.0));
        }

        bool Pose2d::epsilonEquals(const Pose2d &other, double epsilon) const
        {
            return getTranslation().epsilonEquals(other.getTranslation(), epsilon) && getRotation().isParallel(other.getRotation());
        }

        Translation2d Pose2d::intersectionInternal(const Pose2d &a, const Pose2d &b)
        {
            Rotation2d a_r = a.getRotation();
            Rotation2d b_r = b.getRotation();
            Translation2d a_t = a.getTranslation();
            Translation2d b_t = b.getTranslation();

            double tan_b = b_r.tan();
            double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y()) / (a_r.sin() - a_r.cos() * tan_b);
            if (std::isnan(t))
            {
                return Translation2d(ck::math::POS_INF, ck::math::POS_INF);
            }
            return a_t.translateBy(a_r.toTranslation().scale(t));
        }

        Pose2d Pose2d::interpolate(const Pose2d &other, double x) const
        {
            if (x <= 0)
            {
                return Pose2d(*this);
            }
            else if (x >= 1)
            {
                return Pose2d(other);
            }
            Twist2d twist = Pose2d::log(inverse().transformBy(other));
            return transformBy(Pose2d::exp(twist.scaled(x)));
        }

        double Pose2d::distance(const Pose2d &other) const
        {
            return Pose2d::log(inverse().transformBy(other)).norm();
        }

        bool Pose2d::equals(const Pose2d &other) {
            return epsilonEquals(other, ck::math::kEpsilon);
        }

        Pose2d Pose2d::getPose() const
        {
            return *this;
        }

        Pose2d Pose2d::mirror() const
        {
            return Pose2d(Translation2d(getTranslation().x(), -getTranslation().y()), getRotation().inverse());
        }

    } // namespace geometry
} // namespace ck