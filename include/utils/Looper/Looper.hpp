#pragma once

#include <vector>
#include <mutex>
#include <chrono>

#include "frc/Notifier.h"
#include "frc/Timer.h"

#include "../Subsystem.hpp"
#include "Loop.hpp"
#include "../TimeoutTimer.hpp"
#include "../../Constants.hpp"

class Looper : public ILooper, public Reportable {
public:
    static constexpr units::second_t kPeriod = K_LOOPER_DT;

    Looper();

    void registerLoop(Loop & loop) override;

    void start();
    void stop();

    std::vector<void *> generateReport() override;

private:
    bool running_;
    std::vector<Loop*> loops_;
    frc::Notifier notifier_;
    double timestamp_;
    double dt_;
    bool isFirstStart;
    bool isFirstRun;
    std::mutex mtx;

};