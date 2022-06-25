#pragma once
#include <stdexcept>

class AutoModeEndedException : public std::runtime_error
{
public:
    AutoModeEndedException() : std::runtime_error("AUTO MODE DONE!!!! ENDED EARLY!!!!") {}
};