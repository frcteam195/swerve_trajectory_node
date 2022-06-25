#pragma once

#include "ctre/Phoenix.h"
#include <functional>
#include <iostream>

namespace ck
{
    static constexpr int kTalonRetryCount = 3;
    static constexpr int kCANTimeoutMs = 30;

    bool runTalonFunctionWithRetry(std::function<ErrorCode()> func, int id = -1);
}
