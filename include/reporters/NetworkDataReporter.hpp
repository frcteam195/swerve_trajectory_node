#pragma once

#include <vector>
#include <sstream>
#include <string>

#include "frc/Notifier.h"
#include "frc/Timer.h"

#include "NetworkDataTypeBase.hpp"
#include "NetworkDataType.hpp"
#include "DataReporter.hpp"
#include "utils/Singleton.hpp"
#include "Constants.hpp"

namespace ck {
    namespace log {
        class NetworkDataReporter : public DataReporter, public Singleton<NetworkDataReporter> {
            friend Singleton;
        public:
            void registerVariable(NetworkDataTypeBase& logVar) override;

        private:
            NetworkDataReporter();

            std::vector<NetworkDataTypeBase*> reportingSet;
            frc::Notifier notifier_;
            std::mutex mtx;
        };
    }
}