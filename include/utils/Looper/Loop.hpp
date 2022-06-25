#pragma once

#include <string>

class Loop {
public:
    virtual void onFirstStart(double timestamp) = 0;
    virtual void onStart(double timestamp) = 0;
    virtual void onStop(double timestamp) = 0;
    virtual void onLoop(double timestamp) = 0;
    virtual std::string getName() = 0;
    virtual ~Loop() = default;
};
