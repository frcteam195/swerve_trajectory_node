#pragma once

#include "NetworkDataTypeBase.hpp"

namespace ck {
    namespace log {
        class NetworkDataTypeBase;

        class DataReporter {
        public:
            virtual void registerVariable(NetworkDataTypeBase& logVar) = 0;
        };
    }
}