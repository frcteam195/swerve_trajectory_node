#include "utils/ThreadRateControl.hpp"

ThreadRateControl::ThreadRateControl() : m_notifier(HAL_InitializeNotifier(&m_notifierStatus)), eTimer() {}

ThreadRateControl::~ThreadRateControl() {
    HAL_StopNotifier((HAL_NotifierHandle)m_notifier, &m_notifierStatus);
    HAL_CleanNotifier((HAL_NotifierHandle)m_notifier, &m_notifierStatus);
}

void ThreadRateControl::start() {
    if (!started)
    {
        eTimer.start();
        getDt();
        started = true;
    }
    else
    {
        //ConsoleReporter.report("Thread rate control start called too many times!", MessageLevel.ERROR);
    }
}

void ThreadRateControl::start(bool resetStart) {
		if (resetStart)
        {
			started = false;
        }
		start();
}

void ThreadRateControl::doRateControl(int minLoopTimeMs) {
    double remainingTime = ((minLoopTimeMs / 1000.0) - eTimer.hasElapsed());
    if (remainingTime > 0) {
        //Subtract constant offset for code delay (currently 150us)
        HAL_UpdateNotifierAlarm((HAL_NotifierHandle)m_notifier, static_cast<uint64_t>(((frc::Timer::GetFPGATimestamp() + remainingTime) * 1e6) - 150), &m_notifierStatus);
        static_cast<void>(HAL_WaitForNotifierAlarm((HAL_NotifierHandle)m_notifier, &m_notifierStatus)); //Intentionally discard no_discard result
    }
    //mAverageLoopTime.addNumber(eTimer.hasElapsed());
    eTimer.start();
}

double ThreadRateControl::getDt() {
    double currDtCalcTime = frc::Timer::GetFPGATimestamp();
    double dt = currDtCalcTime - prevDtCalcTime;
    prevDtCalcTime = currDtCalcTime;
    return dt;
}