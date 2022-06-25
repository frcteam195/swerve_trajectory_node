#pragma once

#include "utils/ThreadRateControl.hpp"
#include "actions/Action.hpp"

class AutoModeBase
{
private:
    ThreadRateControl threadRateControl;
protected:
    static constexpr double mUpdateRate = 0.010;
    bool mActive;
public:
    virtual void routine() = 0;

    AutoModeBase();

    void run();
    void done();
    void stop();
    bool isActive();
    bool isActiveWithThrow();
    void runAction(Action &action);
};