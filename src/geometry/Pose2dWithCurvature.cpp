#include "geometry/Pose2dWithCurvature.hpp"

namespace ck
{
    namespace geometry
    {
        const Pose2dWithCurvature &Pose2dWithCurvature::identity()
        {
            static Pose2dWithCurvature kIdentity;
            return kIdentity;
        }
        Pose2dWithCurvature::Pose2dWithCurvature() : pose(), curvature(0), dcurvature_ds(0) {}
        Pose2dWithCurvature::Pose2dWithCurvature(const Pose2d &pose, double curvature, double dcurvature_ds)
            : pose(pose), curvature(curvature), dcurvature_ds(dcurvature_ds) {}
        Pose2dWithCurvature::Pose2dWithCurvature(const Translation2d &translation, const Rotation2d &rotation, double curvature, double dcurvature_ds)
            : pose(Pose2d(translation, rotation)), curvature(curvature), dcurvature_ds(dcurvature_ds) {}

        bool Pose2dWithCurvature::operator==(const Pose2dWithCurvature &obj) const
        {
            return (pose == obj.pose) && ck::math::epsilonEquals(curvature, obj.curvature) && ck::math::epsilonEquals(dcurvature_ds, obj.dcurvature_ds);
        }

        std::ostream &operator<<(std::ostream &os, const Pose2dWithCurvature &p2dwc)
        {
            os << std::setprecision(3) << "(P: " << p2dwc.pose << ", C: " << p2dwc.curvature << ", DC: " << p2dwc.dcurvature_ds << ")";
            return os;
        }

        Pose2d Pose2dWithCurvature::getPose() const
        {
            return pose;
        }

        Pose2dWithCurvature Pose2dWithCurvature::transformBy(const Pose2d &transform) const
        {
            return Pose2dWithCurvature(pose.transformBy(transform), curvature, dcurvature_ds);
        }

        Pose2dWithCurvature Pose2dWithCurvature::mirror() const
        {
            return Pose2dWithCurvature(pose.mirror(), -curvature, -dcurvature_ds);
        }

        double Pose2dWithCurvature::getCurvature() const
        {
            return curvature;
        }

        double Pose2dWithCurvature::getDCurvatureDs() const
        {
            return dcurvature_ds;
        }

        Translation2d Pose2dWithCurvature::getTranslation() const
        {
            return pose.getTranslation();
        }

        Rotation2d Pose2dWithCurvature::getRotation() const
        {
            return pose.getRotation();
        }

        Pose2dWithCurvature Pose2dWithCurvature::interpolate(const Pose2dWithCurvature &other, double interpFactor) const
        {
            return Pose2dWithCurvature(pose.interpolate(other.pose, interpFactor),
                                       ck::math::interpolate(curvature, other.curvature, interpFactor),
                                       ck::math::interpolate(dcurvature_ds, other.dcurvature_ds, interpFactor));
        }

        double Pose2dWithCurvature::distance(const Pose2dWithCurvature &other) const
        {
            return pose.distance(other.pose);
        }

        bool Pose2dWithCurvature::equals(const Pose2dWithCurvature &other)
        {
            return getPose().equals(other.getPose()) && ck::math::epsilonEquals(getCurvature(), other.getCurvature()) && ck::math::epsilonEquals(getDCurvatureDs(), other.getDCurvatureDs());
        }
    } // namespace geometry
} // namespace ck