#pragma once

#include <mutex>
#include "ctre/Phoenix.h"

namespace ck {
    class FeedbackConfiguration {
		ctre::phoenix::motorcontrol::FeedbackDevice feedbackDevice;
		int remoteDeviceId;
		double closedLoopRampRate;
		double motionMagicVel;
		double motionMagicAccel;
		int motionMagicSCurveStrength;
        std::mutex feedbackLock;

		FeedbackConfiguration(ctre::phoenix::motorcontrol::FeedbackDevice feedbackDevice, int remoteDeviceId, double closedLoopRampRate, double motionMagicVel, double motionMagicAccel, int motionMagicSCurveStrength) {
			this->feedbackDevice = feedbackDevice;
			this->remoteDeviceId = remoteDeviceId;
			this->closedLoopRampRate = closedLoopRampRate;
			this->motionMagicVel = motionMagicVel;
			this->motionMagicAccel = motionMagicAccel;
			this->motionMagicSCurveStrength = motionMagicSCurveStrength;
		};

		FeedbackConfiguration() {
			FeedbackConfiguration(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative, -1, 0, 0, 0, 0);
		};

		void setFeedbackDevice(ctre::phoenix::motorcontrol::FeedbackDevice feedbackDevice) {
            std::scoped_lock<std::mutex> lock(feedbackLock);
			this->feedbackDevice = feedbackDevice;
		};

		void setRemoteDeviceId(int remoteDeviceId) {
            std::scoped_lock<std::mutex> lock(feedbackLock);
			this->remoteDeviceId = remoteDeviceId;
		};

		void setClosedLoopRampRate(double closedLoopRampRate) {
            std::scoped_lock<std::mutex> lock(feedbackLock);
			this->closedLoopRampRate = closedLoopRampRate;
		};

		void setMotionMagicVel(double motionMagicVel) {
            std::scoped_lock<std::mutex> lock(feedbackLock);
			this->motionMagicVel = motionMagicVel;
		};

		void setMotionMagicAccel(double motionMagicAccel) {
            std::scoped_lock<std::mutex> lock(feedbackLock);
			this->motionMagicAccel = motionMagicAccel;
		};

		void setMotionMagicSCurveStrength(int sCurveStrength) {
            std::scoped_lock<std::mutex> lock(feedbackLock);
			this->motionMagicSCurveStrength = sCurveStrength;
		};
	};
};