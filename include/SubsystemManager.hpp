#pragma once

#include <vector>
#include <initializer_list>

#include "utils/Subsystem.hpp"
#include "utils/Looper/Loop.hpp"
#include "utils/Looper/Looper.hpp"
#include "utils/Singleton.hpp"
#include "utils/TimeoutTimer.hpp"

class SubsystemManager : public ILooper, public Singleton<SubsystemManager> {
    friend Singleton;
public:
    static SubsystemManager& getInstance(std::initializer_list<Subsystem*> subsystemList);
    static SubsystemManager& getInstance() = delete;

    bool checkSystemsPassDiagnostics();

    void registerLoop(Loop& loop) override;

    void registerEnabledLoops(Looper & enabledLooper);

    void registerDisabledLoops(Looper & disabledLooper);

private:
    SubsystemManager();

    static std::vector<Subsystem*> mAllSubsystems;
    static std::vector<Loop*> mLoops;
    static std::vector<Reportable*> mLooperReports;

    friend class EnabledLoop;
    class EnabledLoop : public Loop {
    public:
        EnabledLoop()
        : mCriticalCheckTimeout(0.250) {

        };

        void onFirstStart(double timestamp) override {
            for (Loop* loop : mLoops) {
                loop->onFirstStart(timestamp);
            }
		};

        void onStart(double timestamp) override {
            for (Loop* loop : mLoops) {
                loop->onStart(timestamp);
            }
        };

        void onLoop(double timestamp) override {
            for(Subsystem* subsystem : mAllSubsystems) {
                subsystem->readPeriodicInputs();
            }

            for (Loop* loop : mLoops) {
                loop->onLoop(timestamp);
            }

            for(Subsystem* subsystem : mAllSubsystems) {
                subsystem->writePeriodicOutputs();
            }

			if (mCriticalCheckTimeout.isTimedOut()) {
                for(Subsystem* subsystem : mAllSubsystems) {
                    subsystem->isSystemFaulted();
                }
				mCriticalCheckTimeout.reset();
			}

			// if (Constants.LOGGING_ENABLED) {
			// 	generateReport();
			// 	DataReporter.reportOSCData(boundOSCMesage);
			// }
        };

        void onStop(double timestamp) override {
            for (Loop* loop : mLoops) {
                loop->onStop(timestamp);
            }
        };

        std::string getName() override {
            return "SubsystemManagerEnabledLoop";
        };
    private:
        TimeoutTimer mCriticalCheckTimeout;
    };

    friend class DisabledLoop;
    class DisabledLoop : public Loop {
    public:
        DisabledLoop()
        : mCriticalCheckTimeout(0.250) {

        };

        void onFirstStart(double timestamp) override {};

        void onStart(double timestamp) override {};

        void onLoop(double timestamp) override {
            for(Subsystem* subsystem : mAllSubsystems) {
                subsystem->readPeriodicInputs();
            }

            for(Subsystem* subsystem : mAllSubsystems) {
                subsystem->writePeriodicOutputs();
            }

			if (mCriticalCheckTimeout.isTimedOut()) {
                for(Subsystem* subsystem : mAllSubsystems) {
                    subsystem->isSystemFaulted();
                }
				mCriticalCheckTimeout.reset();
			}

			// if (Constants.LOGGING_ENABLED) {
			// 	generateReport();
			// 	DataReporter.reportOSCData(boundOSCMesage);
			// }
        };

        void onStop(double timestamp) override {};

        std::string getName() override {
            return "SubsystemManagerDisabledLoop";
        };
    private:
        TimeoutTimer mCriticalCheckTimeout;
    };

    EnabledLoop eLoop;
    DisabledLoop dLoop;
};
