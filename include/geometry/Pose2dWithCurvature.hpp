#pragma once

#include "utils/CKMath.hpp"
#include "Pose2d.hpp"
#include "IPose2d.hpp"
#include "ICurvature.hpp"

namespace ck
{
    namespace geometry
    {
        class Pose2dWithCurvature : public IPose2d<Pose2dWithCurvature>, public ICurvature<Pose2dWithCurvature>
        {
        protected:
            Pose2d pose;
            double curvature;
            double dcurvature_ds;

        public:
            static const Pose2dWithCurvature &identity();

            Pose2dWithCurvature();
            Pose2dWithCurvature(const Pose2d &pose, double curvature, double dcurvature_ds = 0);
            Pose2dWithCurvature(const Translation2d &translation, const Rotation2d &rotation, double curvature, double dcurvature_ds = 0);
            virtual ~Pose2dWithCurvature() {};

            bool operator==(const Pose2dWithCurvature &obj) const;
            friend std::ostream &operator<<(std::ostream &os, const Pose2dWithCurvature &t2d);

            Pose2d getPose() const override;
            Pose2dWithCurvature transformBy(const Pose2d &transform) const override;
            Pose2dWithCurvature mirror() const override;
            double getCurvature() const override;
            double getDCurvatureDs() const override;
            Translation2d getTranslation() const override;
            Rotation2d getRotation() const override;
            Pose2dWithCurvature interpolate(const Pose2dWithCurvature &other, double interpFactor) const override;
            double distance(const Pose2dWithCurvature &other) const override;
            bool equals(const Pose2dWithCurvature &other) override;
        };
    } // namespace geometry
} // namespace ck