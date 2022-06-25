#pragma once

#include "frc/Timer.h"

class ElapsedTimer {
public:
    ElapsedTimer();
    void start();
    double hasElapsed();
private:
    double startTime;
};
