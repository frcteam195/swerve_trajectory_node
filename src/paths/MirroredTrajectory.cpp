#include "paths/MirroredTrajectory.hpp"

#include "trajectory/TrajectoryUtil.hpp"

namespace ck
{
    namespace paths
    {
        MirroredTrajectory::MirroredTrajectory(trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature>> right) : right(right)
        {
            this->left = trajectory::TrajectoryUtil::mirrorTimed(right);
        }

        trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature>> MirroredTrajectory::get(bool left)
        {
            return left ? this->left : this->right;
        }
    } // namespace paths
} // namespace ck
