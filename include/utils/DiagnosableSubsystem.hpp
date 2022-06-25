#pragma once

#include <stdbool.h>

class DiagnosableSubsystem {
public:
    virtual bool runDiagnostics() = 0;
    virtual ~DiagnosableSubsystem() = default;
};