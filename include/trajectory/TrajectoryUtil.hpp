#pragma once

#include "ck_utilities/geometry/Pose2dWithCurvature.hpp"
#include "ck_utilities/geometry/QuinticHermiteSpline.hpp"
#include "ck_utilities/geometry/SplineGenerator.hpp"
#include "trajectory/Trajectory.hpp"
#include "trajectory/timing/TimedState.hpp"

namespace ck
{
    namespace trajectory
    {
        class TrajectoryUtil
        {
        public:
            template <class S>
            static Trajectory<S> mirror(Trajectory<S> trajectory)
            {
                std::vector<S> waypoints;

                for (int i = 0; i < trajectory.length(); ++i)
                {
                    waypoints.push_back(trajectory.getState(i).mirror());
                }

                return Trajectory<S>(waypoints);
            }

            template <class S>
            static Trajectory<timing::TimedState<S> > mirrorTimed(Trajectory<timing::TimedState<S> > trajectory)
            {
                std::vector<timing::TimedState<S>> waypoints;

                for (int i = 0; i < trajectory.length(); ++i)
                {
                    timing::TimedState<S> timedState = trajectory.getState(i);
                    waypoints.push_back(timing::TimedState(timedState.state().mirror(), timedState.t(), timedState.velocity(), timedState.acceleration()));
                }

                return Trajectory<timing::TimedState<S>>(waypoints);
            }

            static Trajectory<geometry::Pose2dWithCurvature> trajectoryFromSplines(std::vector<geometry::QuinticHermiteSpline> splines,
                                                                                   double maxDx,
                                                                                   double maxDy,
                                                                                   double maxDtheta)
            {
                std::vector<ck::geometry::Pose2dWithCurvature> splinePoses = geometry::SplineGenerator::parameterizeSplines(splines, maxDx, maxDy, maxDtheta);
                return Trajectory<geometry::Pose2dWithCurvature>(splinePoses);
            }

            static Trajectory<geometry::Pose2dWithCurvature> trajectoryFromSplineWaypoints(std::vector<geometry::Pose2d> waypoints,
                                                                                           double maxDx,
                                                                                           double maxDy,
                                                                                           double maxDtheta);
        };

    } // namespace trajectory

} // namespace ck