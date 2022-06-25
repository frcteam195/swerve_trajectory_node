#pragma once

#include "Reportable.hpp"
#include "CriticalSystemStatus.hpp"
#include "DiagnosableSubsystem.hpp"
#include "Looper/ILooper.hpp"

class Subsystem : public CriticalSystemStatus, public DiagnosableSubsystem {
public:
    virtual void readPeriodicInputs() {};
    virtual void writePeriodicOutputs() {};
    virtual void zeroSensors() {};

    virtual void registerEnabledLoops(ILooper & enabledLooper) {};

    virtual void stop() = 0;

    virtual ~Subsystem() = default;
};
