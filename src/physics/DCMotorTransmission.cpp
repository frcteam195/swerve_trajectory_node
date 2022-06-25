#include "physics/DCMotorTransmission.hpp"

namespace ck
{
    namespace physics
    {
        DCMotorTransmission::DCMotorTransmission(double speed_per_volt, double torque_per_volt, double friction_voltage)
            : speed_per_volt_(speed_per_volt), torque_per_volt_(torque_per_volt), friction_voltage_(friction_voltage) {}

        double DCMotorTransmission::speed_per_volt()
        {
            return speed_per_volt_;
        }

        double DCMotorTransmission::torque_per_volt()
        {
            return torque_per_volt_;
        }

        double DCMotorTransmission::friction_voltage()
        {
            return friction_voltage_;
        }

        double DCMotorTransmission::free_speed_at_voltage(double voltage)
        {
            double absVoltage = std::fabs(voltage);
            if (absVoltage > ck::math::kEpsilon)
            {
                return std::copysignf(ck::math::max(0.0, absVoltage - friction_voltage_) * speed_per_volt_, voltage);
            }
            else
            {
                return 0.0;
            }
        }

        double DCMotorTransmission::getTorqueForVoltage(double output_speed, double voltage)
        {
            double effective_voltage = voltage;
            if (std::fabs(output_speed) > ck::math::kEpsilon)
            {
                // Forward/reverse Motion, rolling friction.
                effective_voltage -= std::copysignf(friction_voltage_, output_speed);
            }
            else if (std::fabs(effective_voltage) > ck::math::kEpsilon)
            {
                // System is static, forward/reverse torque.
                effective_voltage = ck::math::max(0.0, effective_voltage - std::copysignf(friction_voltage_, voltage));
            }
            else
            {
                // System is idle.
                return 0.0;
            }
            return torque_per_volt_ * (-output_speed / speed_per_volt_ + effective_voltage);
        }

        double DCMotorTransmission::getVoltageForTorque(double output_speed, double torque)
        {
            double friction_voltage;
            if (std::fabs(output_speed) > ck::math::kEpsilon)
            {
                // Forward/Reverse motion, rolling friction.
                friction_voltage = std::copysignf(friction_voltage_, output_speed);
            }
            else if (std::fabs(torque) > ck::math::kEpsilon)
            {
                // System is static, forward/reverse torque.
                friction_voltage = std::copysignf(friction_voltage_, torque);
            }
            else
            {
                // System is idle.
                return 0.0;
            }
            return torque / torque_per_volt_ + output_speed / speed_per_volt_ + friction_voltage;
        }

    } // namespace geometry
} // namespace ck