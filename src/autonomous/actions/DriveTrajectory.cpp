#include "autonomous/actions/DriveTrajectory.hpp"
#include "subsystems/Drive.hpp"
#include "frc/Timer.h"

DriveTrajectory::DriveTrajectory( Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
                                  bool resetPose )
{
    mResetPose = resetPose;
    view = new TimedView <Pose2dWithCurvature>  (trajectory);
    mTrajectory = new TrajectoryIterator<TimedState<Pose2dWithCurvature>>(view);
}




DriveTrajectory::DriveTrajectory( Trajectory<TimedState<Pose2dWithCurvature>> trajectory)
    : DriveTrajectory(trajectory, false)
{

}

DriveTrajectory::~DriveTrajectory()
{
    delete mTrajectory;
    delete view;
}


bool DriveTrajectory::isFinished()
{
    if (Drive::getInstance().isDoneWithTrajectory())
    {
        //ConsoleReporter.report("Trajectory finished");
        return true;
    }
    return false;
}

void DriveTrajectory::update() {}
void DriveTrajectory::done() {}
void DriveTrajectory::start()
{
    //ConsoleReporter.report("Starting trajectory! (length=" + FastDoubleToString.format(mTrajectory.getRemainingProgress()) + ")");
    if (mResetPose) {
        //TODO: Finish implementing when RobotState is implemented
        //mRobotState.reset(frc::Timer::GetFPGATimestamp(), mTrajectory.getState().state().getPose());
    }
    Drive::getInstance().setDriveControlState(Drive::DriveControlState::PATH_FOLLOWING);
    //Drive::getInstance().setTrajectory(mTrajectory);
}
