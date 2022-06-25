#include "utils/Looper/Looper.hpp"

Looper::Looper()
: notifier_([this]() {
    std::scoped_lock<std::mutex> lock(mtx);
    if (isFirstRun) {
        //Any init code here
        isFirstRun = false;
    }

    if (running_) {
        double now = frc::Timer::GetFPGATimestamp();
        try {
            for (Loop* loop : loops_) {
                loop->onLoop(now);
            }
        }
        catch (std::exception &ex) {
            //ConsoleReporter.report(ex);
        }
        dt_ = now - timestamp_;
        timestamp_ = now;
    }
}) {
    isFirstRun = true;
    isFirstStart = true;
    running_ = false;
}

void Looper::registerLoop(Loop & loop) {
    std::scoped_lock<std::mutex> lock(mtx);
    loops_.push_back(&loop);
}

void Looper::start() {
    if (!running_) {
        {   //Aquire lock for this scope only
            std::scoped_lock<std::mutex> lock(mtx);
            if (isFirstStart) {
                timestamp_ = frc::Timer::GetFPGATimestamp();
                try {
                    for (Loop* loop : loops_) {
                        loop->onFirstStart(timestamp_);
                    }
                }
                catch (std::exception &ex) {
                    //ConsoleReporter.report(ex);
                }
            }
            timestamp_ = frc::Timer::GetFPGATimestamp();
            try {
                for (Loop* loop : loops_) {
                    loop->onStart(timestamp_);
                }
            }
            catch (std::exception &ex) {
                //ConsoleReporter.report(ex);
            }
            running_ = true;
            isFirstStart = false;
        }

        
        notifier_.StartPeriodic(kPeriod);
    }
}

void Looper::stop() {
    if (running_) {
        notifier_.Stop();
        std::scoped_lock<std::mutex> lock(mtx);
        running_ = false;
        timestamp_ = frc::Timer::GetFPGATimestamp();
        try {
            for (Loop* loop : loops_) {
                loop->onStop(timestamp_);
            }
        }
        catch (std::exception &ex) {
            //ConsoleReporter.report(ex);
        }
    }
}

std::vector<void *> Looper::generateReport() {
    throw std::exception();
}
