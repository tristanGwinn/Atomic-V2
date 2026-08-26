#pragma once

#include "atomic/trajectory/trajectory.h"

#include <algorithm>

using namespace atomic;

bool Trajectory::State::operator==(const Trajectory::State& other) const {
    return t == other.t && linearVelocity == other.linearVelocity &&
           angularVelocity == other.angularVelocity && pose == other.pose;
}

bool Trajectory::State::operator!=(const Trajectory::State& other) const {
    return !operator==(other);
}

Trajectory::State Trajectory::State::interpolate(State endValue,
                                                 double i) const {
    // Find the new [t] value.
    const auto newT = Lerp(t, endValue.t, i);

    // Find the delta time between the current state and the interpolated state.
    const auto deltaT = newT - t;

    // Compute the change in linear velocity over the current and interpolated states
    const auto linearAccel = (endValue.linearVelocity - linearVelocity) / deltaT;

    // Compute the change in angular velocity over the current and interpolated states
    const auto angularAccel = (endValue.angularVelocity - angularVelocity) / deltaT;

    // If delta time is negative, flip the order of interpolation.
    if (deltaT < 0_sec) {
        return endValue.interpolate(*this, 1.0 - i);
    }


    // Check whether the robot is reversing over the interpolation period
    const auto reversing =
        linearVelocity < 0_mps ||
        (fabs(linearVelocity.convert(mps)) < 1E-9 && linearAccel.convert(mps2) < 0);

    // Calculate the new linear velocity.
    // v = v_0 + at
    const LinearVelocity newV = linearVelocity + (linearAccel * deltaT);

    // Calculate the new angular velocity.
    const AngularVelocity newAngularV = angularVelocity + (angularAccel * deltaT);

    // Calculate the change in position.
    // delta_s = v_0 t + 0.5 at^2
    const Length newS =
        (linearVelocity * deltaT + 0.5 * linearAccel * deltaT * deltaT) *
        (reversing ? -1.0 : 1.0);
    
    // Return the new state. To find the new position for the new state, we need
    // to interpolate between the two endpoint poses. The fraction for
    // interpolation is the change in position (delta s) divided by the total
    // distance between the two endpoints.
    const double interpolationFrac =
        newS.convert(m) / endValue.pose.distance(pose);

    return {newT, newV, newAngularV,
            Lerp(pose, endValue.pose, interpolationFrac)};
}

Trajectory::Trajectory(const std::vector<State>& states) : m_states(states) {
    m_totalTime = states.back().t;
}

Trajectory::State Trajectory::sample(Time t) const {
    if (t <= m_states.front().t) {
        return m_states.front();
    }
    if (t >= m_totalTime) {
        return m_states.back();
    }

    // Use binary search to get the element with a timestamp no less than the
    // requested timestamp. This starts at 1 because we use the previous state
    // later on for interpolation.
    auto sample =
        std::lower_bound(m_states.cbegin() + 1, m_states.cend(), t,
                         [](const State& a, const Time& b) { return a.t < b; });

    auto prevSample = sample - 1;

    // The sample's timestamp is now greater than or equal to the requested
    // timestamp. If it is greater, we need to interpolate between the
    // previous state and the current state to get the exact state that we
    // want.

    // If the difference in states is negligible, then we are spot on!
    if (fabs((sample->t - prevSample->t).convert(sec)) < 1E-9) {
        return *sample;
    }
    // Interpolate between the two states for the state that we want.
    return prevSample->interpolate(
        *sample, (t - prevSample->t).convert(sec) / (sample->t - prevSample->t).convert(sec));
}

Trajectory Trajectory::relativeTo(const Pose& pose) {
    Pose translation = Pose(pose.x - m_states.front().pose.x,
                            pose.y - m_states.front().pose.y,
                            pose.theta - m_states.front().pose.theta);

    auto newStates = m_states;
    for (auto& state : newStates) {
        state.pose = Pose(state.pose.x + translation.x,
                          state.pose.y + translation.y,
                          state.pose.theta + translation.theta);
    }
    return Trajectory(newStates);
}

Trajectory Trajectory::operator+(const Trajectory& other) const {
    // If this is a default constructed trajectory with no states, then we can
    // simply return the rhs trajectory.
    if (m_states.empty()) {
        return other;
    }

    auto states = m_states;
    auto otherStates = other.states();
    for (auto& otherState : otherStates) {
        otherState.t += m_totalTime;
    }

    // Here we omit the first state of the other trajectory because we don't want
    // two time points with different states. Sample() will automatically
    // interpolate between the end of this trajectory and the second state of the
    // other trajectory.
    states.insert(states.end(), otherStates.begin() + 1, otherStates.end());
    return Trajectory(states);
}

bool Trajectory::operator==(const Trajectory& other) const {
    return m_states == other.states();
}

bool Trajectory::operator!=(const Trajectory& other) const {
    return !operator==(other);
}