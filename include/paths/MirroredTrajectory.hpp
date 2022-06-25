#pragma once

#include "../geometry/Pose2dWithCurvature.hpp"
#include "../trajectory/Trajectory.hpp"
#include "../trajectory/timing/TimedState.hpp"

namespace ck
{
    namespace paths
    {
        class MirroredTrajectory
        {
        public:
            MirroredTrajectory(trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature> > right);

            trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature> > get(bool left = false);

            trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature> > left;
            trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature> > right;
        };
    } // namespace paths
} // namespace ck
