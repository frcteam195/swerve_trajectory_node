#pragma once

#include <mutex>
#include <functional>
#include "TimeoutTimer.hpp"

template <typename T>
class CachedValue
{
public:
    CachedValue(double updateTimeoutMs, std::function<T&(void)> &updateFunction) {
		mTimeoutTimer = new TimeoutTimer(updateTimeoutMs / 1000.0);
		mUpdateFunction = updateFunction;
	}

    T& getValue() {
		try {
			if (!initialized) {
				setCachedValue(mUpdateFunction());
				initialized = true;
			}

			if (mTimeoutTimer.isTimedOut()) {
				setCachedValue(mUpdateFunction());
				mTimeoutTimer.reset();
			}
		}
		catch (std::exception& ex) {
			std::cout << ex.what() << std::endl();
		}

        std::scoped_lock<std::mutex> lock(cacheLock);
		return mCachedValue;
	}

	void setValue(T& value) {
		setCachedValue(value);
	}

private:
	void setCachedValue(T& value) {
        std::scoped_lock<std::mutex> lock(cacheLock);
		mCachedValue = value;
	}

    TimeoutTimer mTimeoutTimer;
    T mCachedValue;
    bool initialized;
    std::function<T&(void)> mUpdateFunction;
    std::mutex cacheLock;
};