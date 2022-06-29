#pragma once

#include <cmath>
#include <vector>
#include <ostream>
#include <iomanip>
#include "ck_utilities/CKMath.hpp"
#include "utils/PolynomialRegression.hpp"

namespace ck
{
    namespace physics
    {
        struct CharacterizationConstants
        {
            double ks;
            double kv;
            double ka;

            friend std::ostream &operator<<(std::ostream &os, const CharacterizationConstants &constants) {
                os << std::setprecision(10) << "ks: " << constants.ks  << ", kv: " << constants.kv << ", ka: " << constants.ka;
                return os;
            }
        };
        struct VelocityDataPoint
        {
            double velocity;
            double power;
        };
        struct AccelerationDataPoint
        {
            double velocity;
            double power;
            double acceleration;
        };
        struct CurvatureDataPoint
        {
            double linear_velocity;
            double angular_velocity;
            double left_voltage;
            double right_voltage;
        };

        class DriveCharacterization
        {
        public:
            static CharacterizationConstants characterizeDrive(std::vector<VelocityDataPoint> &velocityData, std::vector<AccelerationDataPoint> &accelerationData);
            static void getVelocityCharacterization(std::vector<double> &xPoints, std::vector<double> &yPoints, CharacterizationConstants &constants);
            static void getAccelerationCharacterization(std::vector<double> &xPoints, std::vector<double> &yPoints, CharacterizationConstants &constants);
            static void getVelocityData(std::vector<VelocityDataPoint> &input, std::vector<double> &xPoints, std::vector<double> &yPoints);
            static void getAccelerationData(std::vector<AccelerationDataPoint> &input, CharacterizationConstants &constants, std::vector<double> &xPoints, std::vector<double> &yPoints);
        };
    } // namespace physics
} // namespace ck