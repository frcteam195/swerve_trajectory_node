#include "utils/TimeoutTimer.hpp"
#include <mutex>

TimeoutTimer::TimeoutTimer(double timeout) {
    this->timeout = timeout;
    setFirstRun(true);
}

bool TimeoutTimer::isTimedOut() {
    if (firstRun) {
        eTimer.start();
        setFirstRun(false);
    }
    return eTimer.hasElapsed() > timeout;
}

void TimeoutTimer::reset() {
    setFirstRun(true);
}

double TimeoutTimer::getTimeLeft() {
    return ck::math::max(timeout - eTimer.hasElapsed(), 0.0);
}

void TimeoutTimer::setFirstRun(bool firstRun) {
    std::scoped_lock<std::mutex> lock(mtx);
    this->firstRun = firstRun;
}
