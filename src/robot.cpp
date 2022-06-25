#include "Robot.hpp"

AutoModeExecutor *Robot::mAutoModeExecutor = new AutoModeExecutor();

void Robot::RobotInit() {
    try {
        mDrive = &Drive::getInstance();

        mSubsystemManager = &SubsystemManager::getInstance({
            mDrive,
        });

        mSubsystemManager->registerEnabledLoops(mEnabledLooper);
        mSubsystemManager->registerDisabledLoops(mDisabledLooper);

        ck::paths::TrajectoryGenerator::getInstance().generateTrajectories();

        Drive::getInstance().zeroSensors();
        RobotState::getInstance().reset(frc::Timer::GetFPGATimestamp(), Pose2d::identity());
    } catch (std::exception &ex) {

    }
}

void Robot::RobotPeriodic() {
    // std::cout << "Running robot code periodic!" << std::endl;
}

void Robot::AutonomousInit() {
    try
    {
        mDisabledLooper.start();
        Drive::getInstance().zeroSensors();
        RobotState::getInstance().reset(frc::Timer::GetFPGATimestamp(), Pose2d::identity());
        // mDrive->setBrakeMode(true);
        // mDrive->forceBrakeModeUpdate();
        mEnabledLooper.start();
        
        TestMode testMode;
        mAutoModeExecutor->setAutoMode(testMode);

        if (mAutoModeExecutor->isSet())
        {
            mAutoModeExecutor->start();
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {


}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

void Robot::DisabledInit() {

    
}

void Robot::DisabledPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
