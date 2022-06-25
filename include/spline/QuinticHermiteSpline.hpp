#pragma once

#include <cmath>
#include <vector>
#include <iostream>
#include "utils/CKMath.hpp"
#include "geometry/Geometry.hpp"
#include "Spline.hpp"

namespace ck
{
    namespace spline
    {
        struct ControlPoint
        {
            double ddx;
            double ddy;

            friend std::ostream &operator<<(std::ostream &os, const ControlPoint &cp) {
                os << "cp ddx: " << cp.ddx << ", ddy: " << cp.ddy << std::endl;
                return os;
            }
        };

        class QuinticHermiteSpline : public Spline
        {
        public:
            /**
             * @param p0 The starting pose of the spline
             * @param p1 The ending pose of the spline
             */
            QuinticHermiteSpline(const ck::geometry::Pose2d &p0, const ck::geometry::Pose2d &p1);
            friend std::ostream &operator<<(std::ostream &os, const QuinticHermiteSpline &qhs);

            ck::geometry::Pose2d getStartPose();
            ck::geometry::Pose2d getEndPose();

            /**
             * @param t ranges from 0 to 1
             * @return the point on the spline for that t value
             */
            ck::geometry::Translation2d getPoint(double t) override;
            double getVelocity(double t) override;
            double getCurvature(double t) override;
            double getDCurvature(double t) override;
            ck::geometry::Rotation2d getHeading(double t) override;
            static double sumDCurvature2(std::vector<QuinticHermiteSpline *> &splines);
            static double optimizeSpline(std::vector<QuinticHermiteSpline *> &splines);
            static void runOptimizationIteration(std::vector<QuinticHermiteSpline *> &splines);

        private:
            static constexpr double kEpsilon = 1e-5;
            static constexpr double kStepSize = 1.0;
            static constexpr double kMinDelta = 0.001;
            static constexpr double kSamples = 100.0;
            static constexpr double kMaxIterations = 100.0;

            double x0, x1, dx0, dx1, ddx0, ddx1, y0, y1, dy0, dy1, ddy0, ddy1;
            double ax, bx, cx, dx_, ex, fx, ay, by, cy, dy_, ey, fy;

            /**
             * Used by the curvature optimization function
             */
            QuinticHermiteSpline(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1,
                                 double y0, double y1, double dy0, double dy1, double ddy0, double ddy1);

            /**
             * Re-arranges the spline into an at^5 + bt^4 + ... + f form for simpler computations
             */
            void computeCoefficients();
            double dx(double t);
            double dy(double t);
            double ddx(double t);
            double ddy(double t);
            double dddx(double t);
            double dddy(double t);
            double dCurvature2(double t);
            double sumDCurvature2();
            static double fitParabola(const ck::geometry::Translation2d &p1, const ck::geometry::Translation2d &p2, const ck::geometry::Translation2d &p3);
        };
    } // namespace spline
} // namespace ck