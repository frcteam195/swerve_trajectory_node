#include "autonomous/AutoModeExecutor.hpp"

AutoModeExecutor::AutoModeExecutor() {}

void AutoModeExecutor::setAutoMode(AutoModeBase &new_auto_mode) {
    mAutoMode = &new_auto_mode;
    mThread = std::thread();
}

void AutoModeExecutor::reset() {
    mAutoMode = NULL;
    mThread = std::thread();
}

bool AutoModeExecutor::isSet() {
    return mAutoMode != NULL;
}

void AutoModeExecutor::start() {
    if (mAutoMode != NULL)
    {
        mThread = std::thread
        {
            [&] ()
            {
                if (mAutoMode != NULL) {
                    mAutoMode->run();
                }
            }
        };
    }
}

void AutoModeExecutor::stop() {
    if (mAutoMode != NULL)
    {
        mAutoMode->stop();
    }
}

bool AutoModeExecutor::isRunning() {
    return mThread.joinable() && mAutoMode != NULL;
}

AutoModeBase* AutoModeExecutor::getAutoMode() {
    return mAutoMode;
}