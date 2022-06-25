#pragma once

#include "autonomous/AutoModeBase.hpp"
#include <thread>

class AutoModeExecutor
{
private:
    AutoModeBase *mAutoMode;
    std::thread mThread;
public:
    AutoModeExecutor();
    void setAutoMode(AutoModeBase &new_auto_mode);
    void reset();
    bool isSet();
    void start();
    void stop();
    bool isRunning();
    AutoModeBase* getAutoMode();
};