#pragma once

#include <mutex>
#include "CKMath.hpp"
#include "ElapsedTimer.hpp"

class TimeoutTimer {
public:
    TimeoutTimer(double timeout);

    bool isTimedOut();

    void reset();
    
    double getTimeLeft();

private:
    double timeout;
    bool firstRun;
    ElapsedTimer eTimer;

    std::mutex mtx; 

    void setFirstRun(bool firstRun);
};
