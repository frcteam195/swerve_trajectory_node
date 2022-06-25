#include "spline/SplineGenerator.hpp"

namespace ck
{
    namespace spline
    {
        void SplineGenerator::getSegmentArc(Spline &s, std::vector<ck::geometry::Pose2dWithCurvature> &rv, double t0, double t1, double maxDx, double maxDy, double maxDTheta)
        {
            ck::geometry::Translation2d p0 = s.getPoint(t0);
            ck::geometry::Translation2d p1 = s.getPoint(t1);
            ck::geometry::Rotation2d r0 = s.getHeading(t0);
            ck::geometry::Rotation2d r1 = s.getHeading(t1);
            ck::geometry::Pose2d transformation(ck::geometry::Translation2d(p0, p1).rotateBy(r0.inverse()), r1.rotateBy(r0.inverse()));
            ck::geometry::Twist2d twist = ck::geometry::Pose2d::log(transformation);
            if (twist.dy > maxDy || twist.dx > maxDx || twist.dtheta > maxDTheta)
            {
                getSegmentArc(s, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
                getSegmentArc(s, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
            }
            else
            {
                rv.push_back(s.getPose2dWithCurvature(t1));
            }
        }

        std::vector<ck::geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSpline(Spline &s, double maxDx, double maxDy, double maxDTheta, double t0, double t1)
        {
            std::vector<ck::geometry::Pose2dWithCurvature> rv;
            rv.push_back(s.getPose2dWithCurvature(0.0));
            double dt = (t1 - t0);
            for (double t = 0; t < t1; t += dt / kMinSampleSize)
            {
                getSegmentArc(s, rv, t, t + dt / kMinSampleSize, maxDx, maxDy, maxDTheta);
            }
            return rv;
        }

        std::vector<ck::geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSpline(Spline &s)
        {
            return parameterizeSpline(s, kMaxDX, kMaxDY, kMaxDTheta, 0.0, 1.0);
        }

        std::vector<ck::geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSpline(Spline &s, double maxDx, double maxDy, double maxDTheta)
        {
            return parameterizeSpline(s, maxDx, maxDy, maxDTheta, 0.0, 1.0);
        }

        std::vector<ck::geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSplines(std::vector<Spline *> &splines)
        {
            return parameterizeSplines(splines, kMaxDX, kMaxDY, kMaxDTheta);
        }

        std::vector<ck::geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSplines(std::vector<Spline *> &splines, double maxDx, double maxDy, double maxDTheta)
        {
            std::vector<ck::geometry::Pose2dWithCurvature> rv;
            if (splines.empty())
            {
                return rv;
            }

            rv.push_back(splines[0]->getPose2dWithCurvature(0.0));
            for (Spline *s : splines)
            {
                std::vector<ck::geometry::Pose2dWithCurvature> samples = parameterizeSpline(*s, maxDx, maxDy, maxDTheta);
                samples.erase(samples.begin());

                rv.insert(rv.end(), samples.begin(), samples.end());
            }
            return rv;
        }
    } // namespace spline
} // namespace ck