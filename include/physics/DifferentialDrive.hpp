#pragma once

#include <cmath>
#include <vector>
#include "utils/CKMath.hpp"
#include "utils/PolynomialRegression.hpp"
#include "DCMotorTransmission.hpp"

namespace ck
{
    namespace physics
    {
        struct ChassisState
        {
            double linear;
            double angular;
        };
        struct WheelState
        {
            double left;
            double right;
        };
        struct DriveDynamics
        {
            double curvature = 0.0;            // m^-1
            double dcurvature = 0.0;           // m^-1/m
            ChassisState chassis_velocity;     // m/s
            ChassisState chassis_acceleration; // m/s^2
            WheelState wheel_velocity;         // rad/s
            WheelState wheel_acceleration;     // rad/s^2
            WheelState voltage;                // V
            WheelState wheel_torque;           // N m
        };
        struct MinMax
        {
            double min;
            double max;
        };

        class DifferentialDrive
        {
        protected:
            // All units must be SI!

            // Equivalent mass when accelerating purely linearly, in kg.
            // This is "equivalent" in that it also absorbs the effects of drivetrain inertia.
            // Measure by doing drivetrain acceleration characterization in a straight line.
            double mass_;

            // Equivalent moment of inertia when accelerating purely angularly, in kg*m^2.
            // This is "equivalent" in that it also absorbs the effects of drivetrain inertia.
            // Measure by doing drivetrain acceleration characterization while turning in place.
            double moi_;

            // Drag torque (proportional to angular velocity) that resists turning, in N*m/rad/s
            // Empirical testing of our drivebase showed that there was an unexplained loss in torque ~proportional to angular
            // velocity, likely due to scrub of wheels.
            // NOTE: this may not be a purely linear term, and we have done limited testing, but this factor helps our model to
            // better match reality.  For future seasons, we should investigate what's going on here...
            double angular_drag_;

            // Self-explanatory.  Measure by rolling the robot a known distance and counting encoder ticks.
            double wheel_radius_; // m

            // "Effective" kinematic wheelbase radius.  Might be larger than theoretical to compensate for skid steer.  Measure
            // by turning the robot in place several times and figuring out what the equivalent wheelbase radius is.
            double effective_wheelbase_radius_; // m

            // Transmissions for both sides of the drive.
            DCMotorTransmission left_transmission_;
            DCMotorTransmission right_transmission_;

        public:
            DifferentialDrive(double mass,
                              double moi,
                              double angular_drag,
                              double wheel_radius,
                              double effective_wheelbase_radius,
                              DCMotorTransmission left_transmission,
                              DCMotorTransmission right_transmission);

            double mass();
            double moi();
            double wheel_radius();
            double effective_wheelbase_radius();
            DCMotorTransmission left_transmission();
            DCMotorTransmission right_transmission();
            ChassisState solveForwardKinematics(const WheelState &wheel_motion) const;
            WheelState solveInverseKinematics(const ChassisState &chassis_motion) const;
            DriveDynamics solveForwardDynamics(const ChassisState &chassis_velocity, WheelState voltage);
            DriveDynamics solveForwardDynamics(const WheelState &wheel_velocity, const WheelState voltage);
            void solveForwardDynamics(DriveDynamics &dynamics);
            DriveDynamics solveInverseDynamics(const ChassisState &chassis_velocity, const ChassisState &chassis_acceleration);
            DriveDynamics solveInverseDynamics(const WheelState &wheel_velocity, const WheelState &wheel_acceleration);
            void solveInverseDynamics(DriveDynamics dynamics);
            double getMaxAbsVelocity(double curvature, /*double dcurvature, */ double max_abs_voltage);
            MinMax getMinMaxAcceleration(const ChassisState &chassis_velocity, double curvature, /*double dcurvature,*/ double max_abs_voltage);

        private:
        };
    } // namespace physics
} // namespace ck