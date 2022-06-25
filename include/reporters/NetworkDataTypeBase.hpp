#pragma once

#include <string>
#include <ostream>
#include "DataReporter.hpp"

namespace ck {
    namespace log {
        class DataReporter;

        class NetworkDataTypeBase {
        public:
            std::string dataName;            
            NetworkDataTypeBase(DataReporter* reporter, std::string name);
            void setName(std::string name);
            virtual std::string getReportingValue() const = 0;

            friend std::ostream& operator<<(std::ostream& os, const NetworkDataTypeBase& ndt);
            friend std::ostream& operator<<(std::ostream& os, const NetworkDataTypeBase* ndt);
        };
    }
}