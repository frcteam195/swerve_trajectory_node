#include "spline/QuinticHermiteSpline.hpp"
#include "utils/CKPow.hpp"

namespace ck
{
    namespace spline
    {
        QuinticHermiteSpline::QuinticHermiteSpline(const ck::geometry::Pose2d &p0, const ck::geometry::Pose2d &p1)
        {
            double scale = 1.2 * p0.getTranslation().distance(p1.getTranslation());
            x0 = p0.getTranslation().x();
            x1 = p1.getTranslation().x();
            dx0 = p0.getRotation().cos() * scale;
            dx1 = p1.getRotation().cos() * scale;
            ddx0 = 0;
            ddx1 = 0;
            y0 = p0.getTranslation().y();
            y1 = p1.getTranslation().y();
            dy0 = p0.getRotation().sin() * scale;
            dy1 = p1.getRotation().sin() * scale;
            ddy0 = 0;
            ddy1 = 0;

            computeCoefficients();
        }

        std::ostream &operator<<(std::ostream &os, const QuinticHermiteSpline &qhs)
        {
            os << "ddx0: " << qhs.ddx0 << ", ddx1: " << qhs.ddx1 << ", ddy0: " << qhs.ddy0 << ", ddy1: " << qhs.ddy1 << std::endl;
            return os;
        }

        ck::geometry::Pose2d QuinticHermiteSpline::getStartPose()
        {
            return ck::geometry::Pose2d(ck::geometry::Translation2d(x0, y0), ck::geometry::Rotation2d(dx0, dy0, true));
        }
        ck::geometry::Pose2d QuinticHermiteSpline::getEndPose()
        {
            return ck::geometry::Pose2d(ck::geometry::Translation2d(x1, y1), ck::geometry::Rotation2d(dx1, dy1, true));
        }

        ck::geometry::Translation2d QuinticHermiteSpline::getPoint(double t)
        {
            double x = ax * CKPOW(t,5) + bx * CKPOW(t,4) + cx * CKPOW(t,3) + dx_ * CKPOW(t,2) + ex * t + fx;
            double y = ay * CKPOW(t,5) + by * CKPOW(t,4) + cy * CKPOW(t,3) + dy_ * CKPOW(t,2) + ey * t + fy;
            return ck::geometry::Translation2d(x, y);
        }

        double QuinticHermiteSpline::getVelocity(double t)
        {
            return std::hypot(dx(t), dy(t));
        }

        double QuinticHermiteSpline::getCurvature(double t)
        {
            return (dx(t) * ddy(t) - ddx(t) * dy(t)) / ((dx(t) * dx(t) + dy(t) * dy(t)) * std::sqrt((dx(t) * dx(t) + dy(t) * dy(t))));
        }

        double QuinticHermiteSpline::getDCurvature(double t)
        {
            double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
            double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2 - 3.0 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
            return num / (dx2dy2 * dx2dy2 * std::sqrt(dx2dy2));
        }

        ck::geometry::Rotation2d QuinticHermiteSpline::getHeading(double t)
        {
            return ck::geometry::Rotation2d(dx(t), dy(t), true);
        }

        double QuinticHermiteSpline::sumDCurvature2(std::vector<QuinticHermiteSpline *> &splines)
        {
            double sum = 0;
            for (QuinticHermiteSpline *s : splines)
            {
                sum += s->sumDCurvature2();
            }
            return sum;
        }

        double QuinticHermiteSpline::optimizeSpline(std::vector<QuinticHermiteSpline *> &splines)
        {
            int count = 0;
            double prev = sumDCurvature2(splines);
            // std::cout << prev << std::endl;
            while (count < kMaxIterations)
            {
                runOptimizationIteration(splines);
                double current = sumDCurvature2(splines);

                //if (count > 2 && count < 20) {
                // std::cout << current << std::endl;
                //}

                if (prev - current < kMinDelta)
                {
                    return current;
                }
                prev = current;
                count++;
            }
            return prev;
        }

        void QuinticHermiteSpline::runOptimizationIteration(std::vector<QuinticHermiteSpline *> &splines)
        {
            //can't optimize anything with less than 2 splines
            if (splines.size() <= 1)
            {
                return;
            }

            std::vector<ControlPoint> controlPoints;

#ifdef _MSC_VER
            controlPoints.resize(splines.size() - 1);
#else
            controlPoints.reserve(splines.size() - 1);
#endif
            double magnitude = 0;

            for (size_t i = 0; i < splines.size() - 1; ++i)
            {
                //don't try to optimize colinear points
                if (splines[i]->getStartPose().isColinear(splines[i + 1]->getStartPose()) || splines[i]->getEndPose().isColinear(splines[i + 1]->getEndPose()))
                {
                    continue;
                }
                double original = sumDCurvature2(splines);

                QuinticHermiteSpline *temp = splines[i];
                QuinticHermiteSpline *temp1 = splines[i + 1];
                ControlPoint cp; //holds the gradient at a control point

                //calculate partial derivatives of sumDCurvature2
                splines[i] = new QuinticHermiteSpline(temp->x0, temp->x1, temp->dx0, temp->dx1, temp->ddx0, temp->ddx1 + kEpsilon, temp->y0, temp->y1, temp->dy0, temp->dy1, temp->ddy0, temp->ddy1);
                splines[i + 1] = new QuinticHermiteSpline(temp1->x0, temp1->x1, temp1->dx0, temp1->dx1, temp1->ddx0 + kEpsilon, temp1->ddx1, temp1->y0, temp1->y1, temp1->dy0, temp1->dy1, temp1->ddy0, temp1->ddy1);
                cp.ddx = (sumDCurvature2(splines) - original) / kEpsilon;
                splines[i] = new QuinticHermiteSpline(temp->x0, temp->x1, temp->dx0, temp->dx1, temp->ddx0, temp->ddx1, temp->y0, temp->y1, temp->dy0, temp->dy1, temp->ddy0, temp->ddy1 + kEpsilon);
                splines[i + 1] = new QuinticHermiteSpline(temp1->x0, temp1->x1, temp1->dx0, temp1->dx1, temp1->ddx0, temp1->ddx1, temp1->y0, temp1->y1, temp1->dy0, temp1->dy1, temp1->ddy0 + kEpsilon, temp1->ddy1);
                cp.ddy = (sumDCurvature2(splines) - original) / kEpsilon;

                splines[i] = temp;
                splines[i + 1] = temp1;
                magnitude += cp.ddx * cp.ddx + cp.ddy * cp.ddy;

                // std::cout << splines[i] << splines[i+1] << cp << std::endl;

                controlPoints[i] = cp;
            }

            magnitude = std::sqrt(magnitude);

            //minimize along the direction of the gradient
            //first calculate 3 points along the direction of the gradient
            ck::geometry::Translation2d p2(0, sumDCurvature2(splines)); //middle point is at the current location
            for (size_t i = 0; i < splines.size() - 1; ++i)
            { //first point is offset from the middle location by -stepSize
                if (splines[i]->getStartPose().isColinear(splines[i + 1]->getStartPose()) || splines[i]->getEndPose().isColinear(splines[i + 1]->getEndPose()))
                {
                    continue;
                }
                //normalize to step size
                controlPoints[i].ddx *= kStepSize / magnitude;
                controlPoints[i].ddy *= kStepSize / magnitude;

                //move opposite the gradient by step size amount
                splines[i]->ddx1 -= controlPoints[i].ddx;
                splines[i]->ddy1 -= controlPoints[i].ddy;
                splines[i + 1]->ddx0 -= controlPoints[i].ddx;
                splines[i + 1]->ddy0 -= controlPoints[i].ddy;

                //recompute the spline's coefficients to account for new second derivatives
                splines[i]->computeCoefficients();
                splines[i + 1]->computeCoefficients();
            }
            ck::geometry::Translation2d p1(-kStepSize, sumDCurvature2(splines));

            for (size_t i = 0; i < splines.size() - 1; ++i)
            { //last point is offset from the middle location by +stepSize
                if (splines[i]->getStartPose().isColinear(splines[i + 1]->getStartPose()) || splines[i]->getEndPose().isColinear(splines[i + 1]->getEndPose()))
                {
                    continue;
                }
                //move along the gradient by 2 times the step size amount (to return to original location and move by 1
                // step)
                splines[i]->ddx1 += 2.0 * controlPoints[i].ddx;
                splines[i]->ddy1 += 2.0 * controlPoints[i].ddy;
                splines[i + 1]->ddx0 += 2.0 * controlPoints[i].ddx;
                splines[i + 1]->ddy0 += 2.0 * controlPoints[i].ddy;

                //recompute the spline's coefficients to account for new second derivatives
                splines[i]->computeCoefficients();
                splines[i + 1]->computeCoefficients();
            }
            ck::geometry::Translation2d p3(kStepSize, sumDCurvature2(splines));

            double stepSize = fitParabola(p1, p2, p3); //approximate step size to minimize sumDCurvature2 along the gradient

            for (size_t i = 0; i < splines.size() - 1; ++i)
            {
                if (splines[i]->getStartPose().isColinear(splines[i + 1]->getStartPose()) || splines[i]->getEndPose().isColinear(splines[i + 1]->getEndPose()))
                {
                    continue;
                }
                //move by the step size calculated by the parabola fit (+1 to offset for the final transformation to find
                // p3)
                controlPoints[i].ddx *= 1.0 + stepSize / kStepSize;
                controlPoints[i].ddy *= 1.0 + stepSize / kStepSize;

                splines[i]->ddx1 += controlPoints[i].ddx;
                splines[i]->ddy1 += controlPoints[i].ddy;
                splines[i + 1]->ddx0 += controlPoints[i].ddx;
                splines[i + 1]->ddy0 += controlPoints[i].ddy;

                //recompute the spline's coefficients to account for new second derivatives
                splines[i]->computeCoefficients();
                splines[i + 1]->computeCoefficients();
            }
        }

        QuinticHermiteSpline::QuinticHermiteSpline(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1,
                                                   double y0, double y1, double dy0, double dy1, double ddy0, double ddy1)
            : x0(x0), x1(x1), dx0(dx0), dx1(dx1), ddx0(ddx0), ddx1(ddx1), y0(y0), y1(y1), dy0(dy0), dy1(dy1), ddy0(ddy0), ddy1(ddy1)
        {
            computeCoefficients();
        }

        void QuinticHermiteSpline::computeCoefficients()
        {
            ax = -6.0 * x0 - 3.0 * dx0 - 0.5 * ddx0 + 0.5 * ddx1 - 3.0 * dx1 + 6.0 * x1;
            bx = 15.0 * x0 + 8.0 * dx0 + 1.5 * ddx0 - ddx1 + 7.0 * dx1 - 15.0 * x1;
            cx = -10.0 * x0 - 6.0 * dx0 - 1.5 * ddx0 + 0.5 * ddx1 - 4.0 * dx1 + 10.0 * x1;
            dx_ = 0.5 * ddx0;
            ex = dx0;
            fx = x0;

            ay = -6.0 * y0 - 3.0 * dy0 - 0.5 * ddy0 + 0.5 * ddy1 - 3.0 * dy1 + 6.0 * y1;
            by = 15.0 * y0 + 8.0 * dy0 + 1.5 * ddy0 - ddy1 + 7.0 * dy1 - 15.0 * y1;
            cy = -10.0 * y0 - 6.0 * dy0 - 1.5 * ddy0 + 0.5 * ddy1 - 4.0 * dy1 + 10.0 * y1;
            dy_ = 0.5 * ddy0;
            ey = dy0;
            fy = y0;
        }

        double QuinticHermiteSpline::dx(double t)
        {
            return 5.0 * ax * CKPOW(t,4) + 4.0 * bx * CKPOW(t,3) + 3.0 * cx * CKPOW(t,2) + 2.0 * dx_ * t + ex;
        }

        double QuinticHermiteSpline::dy(double t)
        {
            return 5.0 * ay * CKPOW(t,4) + 4.0 * by * CKPOW(t,3) + 3.0 * cy * CKPOW(t,2) + 2.0 * dy_ * t + ey;
        }

        double QuinticHermiteSpline::ddx(double t)
        {
            return 20.0 * ax * CKPOW(t,3) + 12.0 * bx * CKPOW(t,2) + 6.0 * cx * t + 2.0 * dx_;
        }

        double QuinticHermiteSpline::ddy(double t)
        {
            return 20.0 * ay * CKPOW(t,3) + 12.0 * by * CKPOW(t,2) + 6.0 * cy * t + 2.0 * dy_;
        }

        double QuinticHermiteSpline::dddx(double t)
        {
            return 60.0 * ax * CKPOW(t,2) + 24.0 * bx * t + 6.0 * cx;
        }

        double QuinticHermiteSpline::dddy(double t)
        {
            return 60.0 * ay * CKPOW(t,2) + 24.0 * by * t + 6.0 * cy;
        }

        double QuinticHermiteSpline::dCurvature2(double t)
        {
            double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
            double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2 - 3.0 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
            return num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
        }

        double QuinticHermiteSpline::sumDCurvature2()
        {
            double dt = 1.0 / kSamples;
            double sum = 0;
            for (double t = 0; t < 1.0; t += dt)
            {
                sum += (dt * dCurvature2(t));
            }
            return sum;
        }

        double QuinticHermiteSpline::fitParabola(const ck::geometry::Translation2d &p1, const ck::geometry::Translation2d &p2, const ck::geometry::Translation2d &p3)
        {
            double A = (p3.x() * (p2.y() - p1.y()) + p2.x() * (p1.y() - p3.y()) + p1.x() * (p3.y() - p2.y()));
            double B = (p3.x() * p3.x() * (p1.y() - p2.y()) + p2.x() * p2.x() * (p3.y() - p1.y()) + p1.x() * p1.x() * (p2.y() - p3.y()));
            return -B / (2.0 * A);
        }

    } // namespace spline
} // namespace ck