#pragma once

#include <cmath>
#include <limits>
#include "IRotation2d.hpp"
#include "Translation2d.hpp"
#include "utils/CKMath.hpp"

namespace ck
{
    namespace geometry
    {
        class Translation2d;

        class Rotation2d : public IRotation2d<Rotation2d>
        {
        protected:
            double cos_angle;
            double sin_angle;

        public:
            static const Rotation2d &identity();
            Rotation2d();
            Rotation2d(double x, double y, bool normalize);
            Rotation2d(const Translation2d &direction, bool normalize);

            bool operator==(const Rotation2d &obj) const;
            friend std::ostream &operator<<(std::ostream &os, const Rotation2d &r2d);

            static Rotation2d fromRadians(double angle_radians);
            static Rotation2d fromDegrees(double angle_degrees);

            double cos() const;
            double sin() const;
            double tan() const;
            double getRadians() const;
            double getDegrees() const;
            Rotation2d getRotation() const override;
            Rotation2d rotateBy(const Rotation2d &other) const;
            Rotation2d normal() const;
            Rotation2d inverse() const;
            bool isParallel(const Rotation2d &other) const;
            Translation2d toTranslation() const;
            Rotation2d interpolate(const Rotation2d &other, double interpFactor) const override;
            double distance(const Rotation2d &other) const override;
            bool equals(const Rotation2d &other) override;

        private:
        };
    } // namespace geometry
} // namespace ck