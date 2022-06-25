#include "trajectory/TrajectoryUtil.hpp"

namespace ck
{
    namespace trajectory
    {
        Trajectory<geometry::Pose2dWithCurvature> TrajectoryUtil::trajectoryFromSplineWaypoints(std::vector<geometry::Pose2d> waypoints,
                                                                                                double maxDx,
                                                                                                double maxDy,
                                                                                                double maxDtheta)
        {
            std::vector<spline::QuinticHermiteSpline *> splines;

            for (size_t i = 1; i < waypoints.size(); ++i)
            {
                splines.push_back(new spline::QuinticHermiteSpline(waypoints[i - 1], waypoints[i]));
            }

            spline::QuinticHermiteSpline::optimizeSpline(splines);

            return trajectoryFromSplines(splines, maxDx, maxDy, maxDtheta);
        }

    } // namespace trajectory

} // namespace ck