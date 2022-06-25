#pragma once

#include "autonomous/actions/Action.hpp"
#include "trajectory/TrajectoryIterator.hpp"
#include "trajectory/timing/TimedState.hpp"
#include "geometry/Pose2dWithCurvature.hpp"
#include "trajectory/Trajectory.hpp"
#include "trajectory/TimedView.hpp"

using namespace ck::trajectory;
using namespace ck::trajectory::timing;
using namespace ck::geometry;

class DriveTrajectory : public Action
{
private:

    TrajectoryIterator<TimedState<Pose2dWithCurvature>>* mTrajectory;
    bool mResetPose;
    TimedView<Pose2dWithCurvature> * view;

public:

    DriveTrajectory( Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
                     bool reset );

    DriveTrajectory( Trajectory<TimedState<Pose2dWithCurvature>> trajectory);

    ~DriveTrajectory();

    bool isFinished() override;
    void update() override;
    void done() override;
    void start() override;
};
