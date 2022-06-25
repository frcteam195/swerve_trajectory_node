#include "reporters/NetworkDataReporter.hpp"

namespace ck {
    namespace log {
        NetworkDataReporter::NetworkDataReporter()
        : notifier_([this]() {
            std::ostringstream reportingSummary;
            std::scoped_lock<std::mutex> lock(mtx);

            //TODO: Rework with contiguous memory and dump instead of iteration
            for (NetworkDataTypeBase* ndt : reportingSet) {
                reportingSummary << ndt;
            }

            //TODO: Add code for transmitting stuff periodically
            //printf("%s\n", reportingSummary.str().c_str());
        }) {
            notifier_.StartPeriodic(K_LOG_REPORT_RATE);
        }

        void NetworkDataReporter::registerVariable(NetworkDataTypeBase& logVar) {
            std::scoped_lock<std::mutex> lock(mtx);
            reportingSet.push_back(&logVar);
        }

    }
}