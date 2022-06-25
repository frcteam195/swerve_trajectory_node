#include "subsystems/Drive.hpp"

DataReporter *Drive::logReporter = &NetworkDataReporter::getInstance();

Drive::Drive()
{
}

void Drive::stop()
{
}

void Drive::onFirstStart(double timestamp)
{
}

void Drive::onStart(double timestamp)
{
}

void Drive::onStop(double timestamp)
{
    stop();
}

void Drive::onLoop(double timestamp)
{
}

std::string Drive::getName()
{
    return "DriveLoop";
}

void Drive::registerEnabledLoops(ILooper &enabledLooper)
{
    enabledLooper.registerLoop(*this);
}

double Drive::inchesToRotations(double inches)
{
    return inches / (K_DRIVE_WHEEL_DIAMETER_INCHES * M_PI);
}

double Drive::rotationsToInches(double rotations)
{
    return rotationsToInches * (K_DRIVE_WHEEL_DIAMETER_INCHES * M_PI);
}

bool Drive::isSystemFaulted()
{
    return false;
}

bool Drive::runDiagnostics()
{
    return true;
}

bool Drive::isDoneWithTrajectory()
{
    if (mMotionPlanner == NULL || mDriveControlState != PATH_FOLLOWING)
    {
        return false;
    }

    return mMotionPlanner->isDone() || mOverrideTrajectory;
}

double Drive::getLeftEncoderDistance()
{
    return rotationsToInches(mPeriodicIO.leftPositionRotations);
}

double Drive::getRightEncoderDistance()
{
    return rotationsToInches(mPeriodicIO.rightPositionRotations);
}

double Drive::getRightLinearVelocity()
{
    return rotationsToInches(mPeriodicIO.rightVelocityRpm / 60.0);
}

double Drive::getLeftLinearVelocity()
{
    return rotationsToInches(mPeriodicIO.leftVelocityRpm / 60.0);
}

double Drive::getLinearVelocity()
{
    return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
}

void Drive::setDriveControlState(DriveControlState driveControlState)
{
    std::scoped_lock<std::mutex> lock(memberAccessMtx);
    mDriveControlState = driveControlState;
}

void Drive::setOverrideTrajectory(bool value)
{
    mOverrideTrajectory = value;
}

void Drive::setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory)
{
    std::scoped_lock<std::mutex> lock(memberAccessMtx);
    if (mMotionPlanner != NULL)
    {
        mOverrideTrajectory = false;
        mMotionPlanner->reset();
        mMotionPlanner->setTrajectory(trajectory);
        setDriveControlState(PATH_FOLLOWING);
    }
}

Rotation2d Drive::getHeading()
{
    return mPeriodicIO.gyroHeading;
}

void Drive::setHeading(Rotation2d heading)
{
    mGyroOffset = heading.rotateBy(Rotation2d::fromDegrees(mGyro.getFusedHeading()).inverse());
    mPeriodicIO.gyroHeading = heading;
}

Drive::PeriodicIO::PeriodicIO()
{
    // left_position_rotations = 3;
    // right_position_rotations = 4;
}