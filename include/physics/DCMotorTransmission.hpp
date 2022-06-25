#pragma once

#include <cmath>
#include "utils/CKMath.hpp"

namespace ck
{
    namespace physics
    {
        class DCMotorTransmission
        {
        protected:
            const double speed_per_volt_;   // rad/s per V (no load)
            const double torque_per_volt_;  // N m per V (stall)
            const double friction_voltage_; // V
        public:
            DCMotorTransmission(double speed_per_volt, double torque_per_volt, double friction_voltage);
            double speed_per_volt();
            double torque_per_volt();
            double friction_voltage();
            double free_speed_at_voltage(double voltage);
            double getTorqueForVoltage(double output_speed, double voltage);
            double getVoltageForTorque(double output_speed, double torque);
        };

    } // namespace geometry
} // namespace ck