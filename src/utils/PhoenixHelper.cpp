#include "utils/PhoenixHelper.hpp"

namespace ck
{
    bool runTalonFunctionWithRetry(std::function<ErrorCode()> func, int id)
    {
        ErrorCode eCode;
		int retryCounter = 0;
		do {
			eCode = func();
		} while(eCode != ErrorCode::OK && retryCounter++ < kTalonRetryCount);

		if (retryCounter >= kTalonRetryCount || eCode != ErrorCode::OK)
        {
			std::cout << "Failed to set parameter on Talon ID " << id << " !!!!!!" << std::endl;
            return false;
        }

		return true;
    }
}