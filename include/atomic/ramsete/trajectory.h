#pragma once

#include <vector>

#include "atomic/pose.hpp"
#include "units/units.hpp"
#include "units/Angle.hpp"

namespace atomic {

/**
 * @brief Represents a time-parameterized trajectory that consists of states
 * that represent pose, curvature, time elapsed, velocity, and acceleration
 * at a point.
 */
class Trajectory {
    public:
        /**
         * @brief Represents one point on the trajectory
         */
        struct State {
            // time elapsed since the beginning of the trajectory
            Time t = 0_sec;

            // state velocity
            LinearVelocity velocity = 0_mps;

            // state acceleration
            LinearAcceleration acceleration = 0_mps2;

            // state pose
            Pose pose = Pose(0, 0, 0);

            // state curvature
            Curvature curvature = 0.0_radpm;

            /**
             * @brief Checks equality between this State and another object.
             *
             * @param other The other object.
             * @return Whether the two objects are equal.
             */
            bool operator==(const State& other) const;

            
            /**
             * @brief Checks inequality between this State and another object.
             *
             * @param other The other object.
             * @return Whether the two objects are not equal.
             */
            bool operator!=(const State& other) const;

            /**
             * @brief Interpolates between two States.
             *
             * @param endValue The end value for the interpolation.
             * @param i The interpolant (fraction).
             *
             * @return The interpolated state.
             */
            State interpolate(State endValue, double i) const;

            State(Time ta, LinearVelocity v, LinearAcceleration a, Pose p, Curvature c) :
              t(ta), velocity(v), acceleration(a), pose(p), curvature(c)
            {}

            State() {}

        };

        Trajectory() = default;

        /**
         * @brief constructs a trajectory from a vector of states
         */
        explicit Trajectory(const std::vector<State>& states);

        /**
         * @brief returns the trajectory duration
         * 
         * @return the trajectory duration
         */
        Time totalTime() const { return m_totalTime; }

        /**
         * @brief return the states of a trajectory
         * @return the states of the trajectory
         */
        const std::vector<State>& states() const { return m_states; }

        /**
         * @brief sample the trajectory at a point in time
         * 
         * @param t the point in time in the trajectory to sample
         * @return the state at that point in time
         */
        State sample(Time t) const;

        /**
         * @brief transforms all poses in the trajectory so that
         *        they are relative to the given pose
         * @note this function is helpful for converting a field-relative
         *       trajectory to one that is relative to the robot pose
         * 
         * @param pose the pose that is the orgigin of the coordinate frame
         *             that the current trajectory will be transformed into
         * @return the transformed trajectory
         */
        Trajectory relativeTo(const Pose& pose);

        /**
         * @brief Concatenates another trajectory to the current trajectory.
         * @note The end pose of this trajectory and the start pose of the other one
         *       need to match is a continuous trajectory is desired.
         *
         * @param other The trajectory to concatenate.
         * @return The concatenated trajectory.
         */
        Trajectory operator+(const Trajectory& other) const;

        /**
         * @brief Returns the initial pose of the trajectory.
         *
         * @return The initial pose of the trajectory.
         */
        Pose initialPose() const { return sample(0_sec).pose; }

        /**
         * @brief Checks equality between this Trajectory and another object.
         * 
         * @param other The other object.
         * @return Whether the two objects are equal.
         */
        bool operator==(const Trajectory& other) const;

        /**
         * @brief Checks inequality between this Trajectory and another object.
         *
         * @param other The other object.
         * @return Whether the two objects are inequal.
         */
        bool operator!=(const Trajectory& other) const;

    private:
        std::vector<State> m_states;
        Time m_totalTime = 0_sec;

        /**
         * @brief Linearly interpolates between two values.
         *
         * @param startValue The start value.
         * @param endValue The end value.
         * @param t The fraction for interpolation.
         *
         * @return The interpolated value.
         */
        template <typename T>
        static T Lerp(const T& startValue, const T& endValue, const double t) {
            return startValue + (endValue - startValue) * t;
        }
};

}   // namespace atomic