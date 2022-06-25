#include "utils/ElapsedTimer.hpp"

ElapsedTimer::ElapsedTimer() {}

void ElapsedTimer::start() {
    startTime = frc::Timer::GetFPGATimestamp();
}

double ElapsedTimer::hasElapsed() {
    return frc::Timer::GetFPGATimestamp() - startTime;
}
