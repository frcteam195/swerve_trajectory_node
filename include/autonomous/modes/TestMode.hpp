#pragma once

#include "autonomous/AutoModeBase.hpp"

class TestMode : public AutoModeBase
{
protected:
    void routine() override;
};