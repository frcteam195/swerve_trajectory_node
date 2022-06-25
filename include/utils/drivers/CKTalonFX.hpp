#pragma once

#include <vector>
#include <limits>
#include <mutex>
#include "ctre/Phoenix.h"
#include "PDPBreaker.hpp"
#include "FeedbackConfiguration.hpp"
#include "Configuration.hpp"
#include "utils/CachedValue.hpp"

class CKTalonFX {
private:
    int currentSelectedSlot = 0;
	std::vector<ck::FeedbackConfiguration> mFeedbackConfig;
	double prevOutput = std::numeric_limits<double>::lowest();
	PDPBreaker motorBreaker;
	bool prevForwardLimitVal;
	bool prevReverseLimitVal;

	std::vector<CKTalonFX> followerTalons;

	CachedValue<double> localQuadPosition;

	CachedValue<bool> forwardLimitCachedValue;
	CachedValue<bool> reverseLimitCachedValue;

	bool sensorInverted = false;

	static constexpr ck::Configuration fastMasterConfig = {5, 5, 20};
	static constexpr ck::Configuration normalMasterConfig = {10, 10, 20};
	static constexpr ck::Configuration normalSlaveConfig = {10, 100, 100};

	// MCControlMode currentControlMode = MCControlMode.Disabled;  //Force an update

	double mKa = 0;
	double mKv = 0;
	double mVoltageCompSat = 12;

	double absoluteEncoderOffset = 0;

	double mGearRatioToOutput = 1;

public:
	CKTalonFX() {};
};