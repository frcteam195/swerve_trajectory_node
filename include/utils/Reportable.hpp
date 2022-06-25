#pragma once

#include <vector>

class Reportable {
public:
    //TODO: Rework with loggable types
    virtual std::vector<void *> generateReport() = 0;
    virtual ~Reportable() = default;
};
