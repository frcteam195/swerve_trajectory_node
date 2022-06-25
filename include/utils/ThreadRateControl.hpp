#pragma once

#include "ElapsedTimer.hpp"
#include "frc/Timer.h"
#include "hal/Notifier.h"

class ThreadRateControl {
public:
    ThreadRateControl();
    ~ThreadRateControl();
    void start();
    void start(bool resetStart);
    void doRateControl(int minLoopTimeMs);
    double getDt();

private:
    const HAL_NotifierHandle m_notifier;
    ElapsedTimer eTimer;
    double prevDtCalcTime;
    bool started;
    int m_notifierStatus;

};
