#include "physics/DifferentialDrive.hpp"

namespace ck
{
    namespace physics
    {

        DifferentialDrive::DifferentialDrive(double mass, double moi, double angular_drag, double wheel_radius,
                                             double effective_wheelbase_radius, DCMotorTransmission left_transmission, DCMotorTransmission right_transmission)
            : mass_(mass), moi_(moi), angular_drag_(angular_drag), wheel_radius_(wheel_radius),
              effective_wheelbase_radius_(effective_wheelbase_radius), left_transmission_(left_transmission), right_transmission_(right_transmission) {}

        double DifferentialDrive::mass()
        {
            return mass_;
        }

        double DifferentialDrive::moi()
        {
            return moi_;
        }

        double DifferentialDrive::wheel_radius()
        {
            return wheel_radius_;
        }

        double DifferentialDrive::effective_wheelbase_radius()
        {
            return effective_wheelbase_radius_;
        }

        DCMotorTransmission DifferentialDrive::left_transmission()
        {
            return left_transmission_;
        }

        DCMotorTransmission DifferentialDrive::right_transmission()
        {
            return right_transmission_;
        }

        ChassisState DifferentialDrive::solveForwardKinematics(const WheelState &wheel_motion) const
        {
            ChassisState chassis_motion{wheel_radius_ * (wheel_motion.right + wheel_motion.left) / 2.0,
                                        wheel_radius_ * (wheel_motion.right - wheel_motion.left) / (2.0 * effective_wheelbase_radius_)};
            return chassis_motion;
        }
        WheelState DifferentialDrive::solveInverseKinematics(const ChassisState &chassis_motion) const
        {
            WheelState wheel_motion{(chassis_motion.linear - effective_wheelbase_radius_ * chassis_motion.angular) / wheel_radius_,
                                    (chassis_motion.linear + effective_wheelbase_radius_ * chassis_motion.angular) / wheel_radius_};
            return wheel_motion;
        }
        DriveDynamics DifferentialDrive::solveForwardDynamics(const ChassisState &chassis_velocity, WheelState voltage)
        {
            DriveDynamics dynamics;
            dynamics.wheel_velocity = solveInverseKinematics(chassis_velocity);
            dynamics.chassis_velocity = chassis_velocity;
            dynamics.curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
            if (std::isnan(dynamics.curvature))
            {
                dynamics.curvature = 0.0;
            }
            dynamics.voltage = voltage;
            solveForwardDynamics(dynamics);
            return dynamics;
        }
        DriveDynamics DifferentialDrive::solveForwardDynamics(const WheelState &wheel_velocity, const WheelState voltage)
        {
            DriveDynamics dynamics;
            dynamics.wheel_velocity = wheel_velocity;
            dynamics.chassis_velocity = solveForwardKinematics(wheel_velocity);
            dynamics.curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
            if (std::isnan(dynamics.curvature))
            {
                dynamics.curvature = 0.0;
            }
            dynamics.voltage = voltage;
            solveForwardDynamics(dynamics);
            return dynamics;
        }
        void DifferentialDrive::solveForwardDynamics(DriveDynamics &dynamics)
        {
            bool left_stationary = ck::math::epsilonEquals(dynamics.wheel_velocity.left, 0.0) && std::fabs(dynamics.voltage.left) < left_transmission_.friction_voltage();
            bool right_stationary = ck::math::epsilonEquals(dynamics.wheel_velocity.right, 0.0) && std::fabs(dynamics.voltage.right) < right_transmission_.friction_voltage();
            if (left_stationary && right_stationary)
            {
                // Neither side breaks static friction, so we remain stationary.
                dynamics.wheel_torque.left = dynamics.wheel_torque.right = 0.0;
                dynamics.chassis_acceleration.linear = dynamics.chassis_acceleration.angular = 0.0;
                dynamics.wheel_acceleration.left = dynamics.wheel_acceleration.right = 0.0;
                dynamics.dcurvature = 0.0;
                return;
            }

            // Solve for motor torques generated on each side.
            dynamics.wheel_torque.left = left_transmission_.getTorqueForVoltage(dynamics.wheel_velocity.left, dynamics
                                                                                                                  .voltage.left);
            dynamics.wheel_torque.right = right_transmission_.getTorqueForVoltage(dynamics.wheel_velocity.right, dynamics
                                                                                                                     .voltage.right);

            // Add forces and torques about the center of mass.
            dynamics.chassis_acceleration.linear = (dynamics.wheel_torque.right + dynamics.wheel_torque.left) /
                                                   (wheel_radius_ * mass_);
            // (Tr - Tl) / r_w * r_wb - drag * w = I * angular_accel
            dynamics.chassis_acceleration.angular = effective_wheelbase_radius_ * (dynamics.wheel_torque.right - dynamics.wheel_torque.left) / (wheel_radius_ * moi_) - dynamics.chassis_velocity.angular * angular_drag_ / moi_;

            // Solve for change in curvature from angular acceleration.
            // total angular accel = linear_accel * curvature + v^2 * dcurvature
            dynamics.dcurvature = (dynamics.chassis_acceleration.angular - dynamics.chassis_acceleration.linear * dynamics.curvature) /
                                  (dynamics.chassis_velocity.linear * dynamics.chassis_velocity.linear);
            if (std::isnan(dynamics.dcurvature))
                dynamics.dcurvature = 0.0;

            // Resolve chassis accelerations to each wheel.
            dynamics.wheel_acceleration.left = dynamics.chassis_acceleration.linear - dynamics.chassis_acceleration
                                                                                              .angular *
                                                                                          effective_wheelbase_radius_;
            dynamics.wheel_acceleration.right = dynamics.chassis_acceleration.linear + dynamics.chassis_acceleration
                                                                                               .angular *
                                                                                           effective_wheelbase_radius_;
        }
        DriveDynamics DifferentialDrive::solveInverseDynamics(const ChassisState &chassis_velocity, const ChassisState &chassis_acceleration)
        {
            DriveDynamics dynamics;
            dynamics.chassis_velocity = chassis_velocity;
            dynamics.curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
            if (std::isnan(dynamics.curvature))
                dynamics.curvature = 0.0;
            dynamics.chassis_acceleration = chassis_acceleration;
            dynamics.dcurvature = (dynamics.chassis_acceleration.angular - dynamics.chassis_acceleration.linear * dynamics.curvature) /
                                  (dynamics.chassis_velocity.linear * dynamics.chassis_velocity.linear);
            if (std::isnan(dynamics.dcurvature))
                dynamics.dcurvature = 0.0;
            dynamics.wheel_velocity = solveInverseKinematics(chassis_velocity);
            dynamics.wheel_acceleration = solveInverseKinematics(chassis_acceleration);
            solveInverseDynamics(dynamics);
            return dynamics;
        }
        DriveDynamics DifferentialDrive::solveInverseDynamics(const WheelState &wheel_velocity, const WheelState &wheel_acceleration)
        {
            DriveDynamics dynamics;
            dynamics.chassis_velocity = solveForwardKinematics(wheel_velocity);
            dynamics.curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
            if (std::isnan(dynamics.curvature))
                dynamics.curvature = 0.0;
            dynamics.chassis_acceleration = solveForwardKinematics(wheel_acceleration);
            dynamics.dcurvature = (dynamics.chassis_acceleration.angular - dynamics.chassis_acceleration.linear * dynamics.curvature) /
                                  (dynamics.chassis_velocity.linear * dynamics.chassis_velocity.linear);
            if (std::isnan(dynamics.dcurvature))
                dynamics.dcurvature = 0.0;
            dynamics.wheel_velocity = wheel_velocity;
            dynamics.wheel_acceleration = wheel_acceleration;
            solveInverseDynamics(dynamics);
            return dynamics;
        }
        void DifferentialDrive::solveInverseDynamics(DriveDynamics dynamics)
        {
            // Determine the necessary torques on the left and right wheels to produce the desired wheel accelerations.
            dynamics.wheel_torque.left = wheel_radius_ / 2.0 * (dynamics.chassis_acceleration.linear * mass_ - dynamics.chassis_acceleration.angular * moi_ / effective_wheelbase_radius_ - dynamics.chassis_velocity.angular * angular_drag_ / effective_wheelbase_radius_);
            dynamics.wheel_torque.right = wheel_radius_ / 2.0 * (dynamics.chassis_acceleration.linear * mass_ + dynamics.chassis_acceleration.angular * moi_ / effective_wheelbase_radius_ + dynamics.chassis_velocity.angular * angular_drag_ / effective_wheelbase_radius_);

            // Solve for input voltages.
            dynamics.voltage.left = left_transmission_.getVoltageForTorque(dynamics.wheel_velocity.left, dynamics.wheel_torque.left);
            dynamics.voltage.right = right_transmission_.getVoltageForTorque(dynamics.wheel_velocity.right, dynamics.wheel_torque.right);
        }
        double DifferentialDrive::getMaxAbsVelocity(double curvature, /*double dcurvature, */ double max_abs_voltage)
        {
            // Alternative implementation:
            // (Tr - Tl) * r_wb / r_w = I * v^2 * dk
            // (Tr + Tl) / r_w = 0
            // T = Tr = -Tl
            // 2T * r_wb / r_w = I*v^2*dk
            // T = 2*I*v^2*dk*r_w/r_wb
            // T = kt*(-vR/kv + V) = -kt*(-vL/vmax + V)
            // Vr = v * (1 + k*r_wb)
            // 0 = 2*I*dk*r_w/r_wb * v^2 + kt * ((1 + k*r_wb) * v / kv) - kt * V
            // solve using quadratic formula?
            // -b +/- sqrt(b^2 - 4*a*c) / (2a)

            // k = w / v
            // v = r_w*(wr + wl) / 2
            // w = r_w*(wr - wl) / (2 * r_wb)
            // Plug in max_abs_voltage for each wheel.
            double left_speed_at_max_voltage = left_transmission_.free_speed_at_voltage(max_abs_voltage);
            double right_speed_at_max_voltage = right_transmission_.free_speed_at_voltage(max_abs_voltage);
            if (ck::math::epsilonEquals(curvature, 0.0))
            {
                return wheel_radius_ * ck::math::min(left_speed_at_max_voltage, right_speed_at_max_voltage);
            }
            if (std::isinf(curvature))
            {
                // Turn in place.  Return value meaning becomes angular velocity.
                double wheel_speed = ck::math::min(left_speed_at_max_voltage, right_speed_at_max_voltage);
                return ck::math::signum(curvature) * wheel_radius_ * wheel_speed / effective_wheelbase_radius_;
            }

            double right_speed_if_left_max = left_speed_at_max_voltage * (effective_wheelbase_radius_ * curvature + 1.0) / (1.0 - effective_wheelbase_radius_ * curvature);
            if (std::fabs(right_speed_if_left_max) <= right_speed_at_max_voltage + ck::math::kEpsilon)
            {
                // Left max is active constraint.
                return wheel_radius_ * (left_speed_at_max_voltage + right_speed_if_left_max) / 2.0;
            }
            double left_speed_if_right_max = right_speed_at_max_voltage * (1.0 - effective_wheelbase_radius_ * curvature) / (1.0 + effective_wheelbase_radius_ * curvature);
            // Right at max is active constraint.
            return wheel_radius_ * (right_speed_at_max_voltage + left_speed_if_right_max) / 2.0;
        }

        MinMax DifferentialDrive::getMinMaxAcceleration(const ChassisState &chassis_velocity, double curvature, /*double dcurvature,*/ double max_abs_voltage)
        {
            WheelState wheel_velocities = solveInverseKinematics(chassis_velocity);
            MinMax result{ck::math::POS_INF, ck::math::NEG_INF};

            // Math:
            // (Tl + Tr) / r_w = m*a
            // (Tr - Tl) / r_w * r_wb - drag*w = i*(a * k + v^2 * dk)

            // 2 equations, 2 unknowns.
            // Solve for a and (Tl|Tr)

            double linear_term = std::isinf(curvature) ? 0.0 : mass_ * effective_wheelbase_radius_;
            double angular_term = std::isinf(curvature) ? moi_ : moi_ * curvature;

            double drag_torque = chassis_velocity.angular * angular_drag_;

            // Check all four cases and record the min and max valid accelerations.
            for (int i = 0; i < 2; i++)
            {
                bool left = i == 0;
                for (int j = 0; j < 2; j++)
                {
                    double sign = j == 0 ? 1 : -1;
                    DCMotorTransmission fixed_transmission = left ? left_transmission_ : right_transmission_;
                    DCMotorTransmission variable_transmission = left ? right_transmission_ : left_transmission_;
                    double fixed_torque = fixed_transmission.getTorqueForVoltage(left ? wheel_velocities.left : wheel_velocities.right, sign *
                                                                                                                                            max_abs_voltage);
                    double variable_torque = 0.0;
                    // NOTE: variable_torque is wrong.  Units don't work out correctly.  We made a math error somewhere...
                    // Leaving this "as is" for code release so as not to be disingenuous, but this whole function needs
                    // revisiting in the future...
                    if (left)
                    {
                        variable_torque = ((/*-moi_ * chassis_velocity.linear * chassis_velocity.linear * dcurvature*/ -drag_torque) * mass_ * wheel_radius_ + fixed_torque *
                                                                                                                                                                   (linear_term + angular_term)) /
                                          (linear_term - angular_term);
                    }
                    else
                    {
                        variable_torque = ((/*moi_ * chassis_velocity.linear * chassis_velocity.linear * dcurvature*/ +drag_torque) * mass_ * wheel_radius_ + fixed_torque *
                                                                                                                                                                  (linear_term - angular_term)) /
                                          (linear_term + angular_term);
                    }
                    double variable_voltage = variable_transmission.getVoltageForTorque(!left ? wheel_velocities.left : wheel_velocities.right, variable_torque);
                    if (std::fabs(variable_voltage) <= max_abs_voltage + ck::math::kEpsilon)
                    {
                        double accel = 0.0;
                        if (std::isinf(curvature))
                        {
                            accel = (left ? -1.0 : 1.0) * (fixed_torque - variable_torque) * effective_wheelbase_radius_ / (moi_ * wheel_radius_) - drag_torque / moi_ /*- chassis_velocity.linear * chassis_velocity.linear * dcurvature*/;
                        }
                        else
                        {
                            accel = (fixed_torque + variable_torque) / (mass_ * wheel_radius_);
                        }
                        result.min = ck::math::min(result.min, accel);
                        result.max = ck::math::max(result.max, accel);
                    }
                }
            }
            return result;
        }

    } // namespace physics
} // namespace ck