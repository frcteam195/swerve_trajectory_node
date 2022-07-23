#pragma once

#include "ck_utilities/CKMath.hpp"
#include "ck_utilities/geometry/Geometry.hpp"
#include "trajectory/timing/MinMaxAcceleration.hpp"
#include "trajectory/Trajectory.hpp"
#include "trajectory/DistanceView.hpp"
#include "TimingConstraint.hpp"
#include "TimedState.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <type_traits>
#include <vector>

namespace ck
{
    namespace trajectory
    {
        namespace timing
        {
            template <class S>
            struct ConstrainedState
            {
                S state;
                double distance;
                double max_velocity;
                double min_acceleration;
                double max_acceleration;
            };

            class TimingUtil
            {
                // static_assert(std::is_base_of<ck::geometry::State<S>, S>::value, "S must inherit from State<S>");
            public:
                template <class S>
                static ck::trajectory::Trajectory<TimedState<S>> timeParameterizeTrajectory(
                    bool reverse,
                    ck::trajectory::DistanceView<S> &distance_view,
                    double step_size,
                    std::vector<TimingConstraint<S>> &constraints,
                    double start_velocity,
                    double end_velocity,
                    double max_velocity,
                    double max_abs_acceleration)
                {
                    int num_states = (int)std::ceil(distance_view.last_interpolant() / step_size + 1);
                    std::vector<S> states;
                    for (int i = 0; i < num_states; ++i)
                    {
                        states.push_back(distance_view.sample(ck::math::min(i * step_size, distance_view.last_interpolant())).state_);
                    }
                    return timeParameterizeTrajectory(reverse, states, constraints, start_velocity, end_velocity, max_velocity, max_abs_acceleration);
                }

                template <class S>
                static ck::trajectory::Trajectory<TimedState<S>> timeParameterizeTrajectory(
                    bool reverse,
                    std::vector<S> &states,
                    std::vector<TimingConstraint<S>> &constraints,
                    double start_velocity,
                    double end_velocity,
                    double max_velocity,
                    double max_abs_acceleration)
                {
                    std::vector<ConstrainedState<S>> constraint_states;
                    constraint_states.reserve(states.size());
                    constexpr double kEpsilon = 1e-6;

                    // Forward pass. We look at pairs of consecutive states, where the start state has already been velocity
                    // parameterized (though we may adjust the velocity downwards during the backwards pass). We wish to find an
                    // acceleration that is admissible at both the start and end state, as well as an admissible end velocity. If
                    // there is no admissible end velocity or acceleration, we set the end velocity to the state's maximum allowed
                    // velocity and will repair the acceleration during the backward pass (by slowing down the predecessor).
                    ConstrainedState<S> predecessor;
                    predecessor.state = states[0];
                    predecessor.distance = 0.0;
                    predecessor.max_velocity = start_velocity;
                    predecessor.min_acceleration = -max_abs_acceleration;
                    predecessor.max_acceleration = max_abs_acceleration;
                    for (size_t i = 0; i < states.size(); ++i)
                    {
                        // Add the new state.
                        ConstrainedState<S> constraint_state;
                        constraint_state.state = states[i];
                        double ds = constraint_state.state.distance(predecessor.state);
                        constraint_state.distance = ds + predecessor.distance;

                        // We may need to iterate to find the maximum end velocity and common acceleration, since acceleration
                        // limits may be a function of velocity.
                        while (true)
                        {
                            // Enforce global max velocity and max reachable velocity by global acceleration limit.
                            // vf = sqrt(vi^2 + 2*a*d)
                            constraint_state.max_velocity = ck::math::min(max_velocity, std::sqrt(predecessor.max_velocity * predecessor.max_velocity + 2.0 * predecessor.max_acceleration * ds));
                            if (std::isnan(constraint_state.max_velocity))
                            {
                                throw;
                            }
                            // Enforce global max absolute acceleration.
                            constraint_state.min_acceleration = -max_abs_acceleration;
                            constraint_state.max_acceleration = max_abs_acceleration;

                            // At this point, the state is full constructed, but no constraints have been applied aside from
                            // predecessor
                            // state max accel.

                            // Enforce all velocity constraints.
                            for (const TimingConstraint<S> &constraint : constraints)
                            {
                                constraint_state.max_velocity = ck::math::min(constraint_state.max_velocity, constraint.getMaxVelocity(constraint_state.state));
                            }
                            if (constraint_state.max_velocity < 0.0)
                            {
                                // This should never happen if constraints are well-behaved.
                                throw;
                            }

                            // Now enforce all acceleration constraints.
                            for (const TimingConstraint<S> &constraint : constraints)
                            {
                                MinMaxAcceleration min_max_accel = constraint.getMinMaxAcceleration(constraint_state.state, (reverse ? -1.0 : 1.0) * constraint_state.max_velocity);
                                if (!min_max_accel.valid())
                                {
                                    // This should never happen if constraints are well-behaved.
                                    throw;
                                }
                                constraint_state.min_acceleration = ck::math::max(constraint_state.min_acceleration, reverse ? -min_max_accel.max_acceleration() : min_max_accel.min_acceleration());
                                constraint_state.max_acceleration = ck::math::min(constraint_state.max_acceleration, reverse ? -min_max_accel.min_acceleration() : min_max_accel.max_acceleration());
                            }
                            if (constraint_state.min_acceleration > constraint_state.max_acceleration)
                            {
                                // This should never happen if constraints are well-behaved.
                                throw;
                            }

                            if (ds < kEpsilon)
                            {
                                break;
                            }
                            // If the max acceleration for this constraint state is more conservative than what we had applied, we
                            // need to reduce the max accel at the predecessor state and try again.
                            // TODO: Simply using the new max acceleration is guaranteed to be valid, but may be too conservative.
                            // Doing a search would be better.
                            double actual_acceleration = (constraint_state.max_velocity * constraint_state.max_velocity - predecessor.max_velocity * predecessor.max_velocity) / (2.0 * ds);
                            if (constraint_state.max_acceleration < actual_acceleration - kEpsilon)
                            {
                                predecessor.max_acceleration = constraint_state.max_acceleration;
                            }
                            else
                            {
                                if (actual_acceleration > predecessor.min_acceleration + kEpsilon)
                                {
                                    predecessor.max_acceleration = actual_acceleration;
                                }
                                // If actual acceleration is less than predecessor min accel, we will repair during the backward
                                // pass.
                                break;
                            }
                            // ConsoleReporter.report("(intermediate) i: " + i + ", " + constraint_state.toString());
                        }
                        // ConsoleReporter.report("i: " + i + ", " + constraint_state.toString());
                        predecessor = constraint_state;
                        constraint_states.push_back(constraint_state);
                    }

                    // Backward pass.
                    ConstrainedState<S> successor;
                    successor.state = states[states.size() - 1];
                    successor.distance = constraint_states[states.size() - 1].distance;
                    successor.max_velocity = end_velocity;
                    successor.min_acceleration = -max_abs_acceleration;
                    successor.max_acceleration = max_abs_acceleration;
                    for (int i = states.size() - 1; i >= 0; --i)
                    {
                        ConstrainedState<S> &constraint_state = constraint_states[i];
                        double ds = constraint_state.distance - successor.distance; // will be negative.

                        while (true)
                        {
                            // Enforce reverse max reachable velocity limit.
                            // vf = sqrt(vi^2 + 2*a*d), where vi = successor.
                            double new_max_velocity = std::sqrt(successor.max_velocity * successor.max_velocity + 2.0 * successor.min_acceleration * ds);
                            if (new_max_velocity >= constraint_state.max_velocity)
                            {
                                // No new limits to impose.
                                break;
                            }
                            constraint_state.max_velocity = new_max_velocity;
                            if (std::isnan(constraint_state.max_velocity))
                            {
                                throw;
                            }

                            // Now check all acceleration constraints with the lower max velocity.
                            for (const TimingConstraint<S> &constraint : constraints)
                            {
                                MinMaxAcceleration min_max_accel = constraint.getMinMaxAcceleration(constraint_state.state, (reverse ? -1.0 : 1.0) * constraint_state.max_velocity);
                                if (!min_max_accel.valid())
                                {
                                    throw;
                                }
                                constraint_state.min_acceleration = ck::math::max(constraint_state.min_acceleration, reverse ? -min_max_accel.max_acceleration() : min_max_accel.min_acceleration());
                                constraint_state.max_acceleration = ck::math::min(constraint_state.max_acceleration, reverse ? -min_max_accel.min_acceleration() : min_max_accel.max_acceleration());
                            }
                            if (constraint_state.min_acceleration > constraint_state.max_acceleration)
                            {
                                throw;
                            }

                            if (ds > kEpsilon)
                            {
                                break;
                            }
                            // If the min acceleration for this constraint state is more conservative than what we have applied, we
                            // need to reduce the min accel and try again.
                            // TODO: Simply using the new min acceleration is guaranteed to be valid, but may be too conservative.
                            // Doing a search would be better.
                            double actual_acceleration = (constraint_state.max_velocity * constraint_state.max_velocity - successor.max_velocity * successor.max_velocity) / (2.0 * ds);
                            if (constraint_state.min_acceleration > actual_acceleration + kEpsilon)
                            {
                                successor.min_acceleration = constraint_state.min_acceleration;
                            }
                            else
                            {
                                successor.min_acceleration = actual_acceleration;
                                break;
                            }
                        }
                        successor = constraint_state;
                    }

                    // Integrate the constrained states forward in time to obtain the TimedStates.
                    std::vector<TimedState<S>> timed_states;
                    timed_states.reserve(states.size());
                    double t = 0.0;
                    double s = 0.0;
                    double v = 0.0;
                    for (size_t i = 0; i < states.size(); ++i)
                    {
                        ConstrainedState<S> &constrained_state = constraint_states[i];
                        // Advance t.
                        double ds = constrained_state.distance - s;
                        double accel = (constrained_state.max_velocity * constrained_state.max_velocity - v * v) / (2.0 * ds);
                        double dt = 0.0;
                        if (i > 0)
                        {
                            timed_states[i - 1].set_acceleration(reverse ? -accel : accel);
                            if (std::fabs(accel) > kEpsilon)
                            {
                                dt = (constrained_state.max_velocity - v) / accel;
                            }
                            else if (std::fabs(v) > kEpsilon)
                            {
                                dt = ds / v;
                            }
                            else
                            {
                                throw std::runtime_error("Value for accel and velocity is zero in TimingUtil");
                            }
                        }
                        t += dt;
                        if (std::isnan(t) || std::isinf(t))
                        {
                            throw std::runtime_error("Value for time is infinite in TimingUtil");
                        }

                        v = constrained_state.max_velocity;
                        s = constrained_state.distance;
                        timed_states.push_back(TimedState<S>(constrained_state.state, t, reverse ? -v : v, reverse ? -accel : accel));
                    }
                    return Trajectory<TimedState<S>>(timed_states);
                }
            };
        } // namespace timing
    }     // namespace trajectory
} // namespace ck