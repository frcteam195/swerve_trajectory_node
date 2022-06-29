#pragma once

#include "ck_utilities/CKMath.hpp"
#include "ck_utilities/geometry/State.hpp"

#include <type_traits>

namespace ck
{
    namespace trajectory
    {
        namespace timing
        {
            template <class S>
            class TimedState : public ck::geometry::State<TimedState<S>>
            {
                static_assert(std::is_base_of<ck::geometry::State<S>, S>::value, "S must inherit from State<S>");

            protected:
                S state_;
                double t_;
                double velocity_;
                double acceleration_;

            public:
                TimedState(S state) : state_(state) {}
                TimedState(S state, double t, double velocity, double acceleration) : state_(state), t_(t), velocity_(velocity), acceleration_(acceleration) {}
                virtual ~TimedState() {}
                bool operator==(const TimedState<S> &obj) const { return state().equals(obj.state()) && ck::math::epsilonEquals(t(), obj.t()); }
                S state() const { return state_; }
                void set_t(double t) { t_ = t; }
                double t() const { return t_; }
                void set_velocity(double velocity) { velocity_ = velocity; }
                double velocity() const { return velocity_; }
                void set_acceleration(double acceleration) { acceleration_ = acceleration; }
                double acceleration() const { return acceleration_; }
                TimedState<S> interpolate(const TimedState<S> &other, double x) const override
                {
                    double new_t = ck::math::interpolate(t(), other.t(), x);
                    double delta_t = new_t - t();
                    if (delta_t < 0.0)
                    {
                        return other.interpolate(*this, 1.0 - x);
                    }
                    bool reversing = velocity() < 0.0 || (ck::math::epsilonEquals(velocity(), 0.0) && acceleration() < 0.0);
                    double new_v = velocity() + acceleration() * delta_t;
                    double new_s = (reversing ? -1.0 : 1.0) * (velocity() * delta_t + .5 * acceleration() * delta_t * delta_t);
                    return TimedState<S>(state().interpolate(other.state(), new_s / state().distance(other.state())), new_t, new_v, acceleration());
                }

                double distance(const TimedState<S> &other) const override
                {
                    return state().distance(other.state());
                }

                bool equals(const TimedState<S> &other) override
                {
                    return *this == other;
                }
            };
        } // namespace timing
    }     // namespace trajectory
} // namespace ck