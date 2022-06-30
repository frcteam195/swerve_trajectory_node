#pragma once

#include "ck_utilities/CKMathConstants.hpp"
#include "ck_utilities/geometry/Geometry.hpp"
#include "MinMaxAcceleration.hpp"
#include "TimingConstraint.hpp"
#include "ck_utilities/physics/DifferentialDrive.hpp"

#include <cmath>
#include <type_traits>

namespace ck
{
    namespace trajectory
    {
        namespace timing
        {
            template <class S>
            class DifferentialDriveDynamicsConstraint : public TimingConstraint<S>
            {
                static_assert(std::is_base_of<ck::geometry::IPose2d<S>, S>::value, "S must inherit from IPose2d<S>");
                static_assert(std::is_base_of<ck::geometry::ICurvature<S>, S>::value, "S must inherit from ICurvature<S>");

            protected:
                ck::physics::DifferentialDrive *drive_;
                double abs_voltage_limit_;

            public:
                DifferentialDriveDynamicsConstraint(physics::DifferentialDrive &drive, double abs_voltage_limit) : drive_(&drive), abs_voltage_limit_(abs_voltage_limit) {}

                double getMaxVelocity(const S &state) const
                {
                    return ck::math::meters_to_inches(drive_->getMaxAbsVelocity(
                        ck::math::meters_to_inches(state.getCurvature()), // Curvature is in inverse inches, so meters_to_inches is correct.
                        /*Units.meters_to_inches(Units.meters_to_inches(state.getDCurvatureDs())),  // DCurvature is in inverse inches^2.*/
                        abs_voltage_limit_));
                }

                MinMaxAcceleration getMinMaxAcceleration(const S &state, double velocity)
                {
                    // TODO figure out a units convention for generic states.  Traditionally we use inches...
                    // NOTE: units cancel on angular velocity.
                    ck::physics::MinMax min_max =
                        drive_->getMinMaxAcceleration(ck::physics::ChassisState{ck::math::inches_to_meters(velocity), state.getCurvature() * velocity},
                                                     ck::math::meters_to_inches(state.getCurvature()), // Curvature is in inverse inches, so meters_to_inches is correct.
                                                     /*Units.meters_to_inches(Units.meters_to_inches(state.getDCurvatureDs())),  // DCurvature is in inverse inches^2.*/
                                                     abs_voltage_limit_);
                    return MinMaxAcceleration(ck::math::meters_to_inches(min_max.min), ck::math::meters_to_inches(min_max.max));
                }
            };
        } // namespace timing
    }     // namespace trajectory
} // namespace ck