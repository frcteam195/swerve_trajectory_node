#pragma once

#include "Loop.hpp"

class ILooper {
public:
    virtual void registerLoop(Loop & loop) = 0;
    virtual ~ILooper() = default;
};
