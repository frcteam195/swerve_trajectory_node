#pragma once

#include <string>
// #include <frc/livewindow/LiveWindow.h>
#include <iostream>
#include <frc/TimedRobot.h>
#include "frc/Timer.h"

#include "SubsystemManager.hpp"
#include "utils/Looper/Looper.hpp"
#include "reporters/NetworkDataType.hpp"

#include "autonomous/AutoModeExecutor.hpp"
#include "autonomous/modes/TestMode.hpp"
#include "subsystems/Drive.hpp"
#include "paths/TrajectoryGenerator.hpp"
#include "RobotState.hpp"

class Robot : public frc::TimedRobot {
public:
    void RobotInit() override;
    void RobotPeriodic() override;  
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;

    static AutoModeExecutor *mAutoModeExecutor; 

private:
    SubsystemManager* mSubsystemManager;
    Looper mEnabledLooper;
    Looper mDisabledLooper;

    Drive *mDrive;
};
