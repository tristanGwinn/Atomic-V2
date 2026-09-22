#pragma once

#include "units/units.hpp"
#include <optional>

/**
 * @brief Struct to hold PID gains.
 *
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 */
struct Gains {
        double kP = 0;
        double kI = 0;
        double kD = 0;
};

class PID {
    public:
        /**
         * @brief Construct a new PID
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         * @param windupRange integral anti windup range
         * @param signFlipReset whether to reset integral when sign of error flips
         */
        PID(const double kP, const double kI, const double kD, const double windupRange, bool signFlipReset)
         : m_gains({kP, kI, kD}), m_windupRange(windupRange), m_signFlipReset(signFlipReset) {}

        /**
         * @brief Constructs a new PID controller
         *
         * @param gains the gains to use
         * @param windupRange range at which integral is reset
         * @param signFlipReset whether to reset integral when error changes sign
         */
        PID(const Gains& gains, Number windupRange, bool signFlipReset)
         : m_gains(gains), m_windupRange(windupRange), m_signFlipReset(signFlipReset) {}

        /**
         * @brief Get the current gains
         *
         * @return Gains the current gains
         */
        Gains getGains() { return m_gains; }

        /**
         * @brief Set the new gains
         *
         * @param gains the new gains
         */
        void setGains(Gains gains) { m_gains = gains; }

        /**
         * @brief Set the target
         *
         * @param target the new target
         */
        void setTarget(Number target) { m_target = target; }

        /**
         * @brief Updates the PID controller using a given error, and outputs the next control signal.
         *
         * @param input current position
         * @return The control signal (output)
         */
        Number update(Number input){
            m_error = m_target - input;     // get error

            // find delta time (dt scaling is from lemlib; should give more consistent output)
            const Time now = from_msec(pros::millis());
            // if first iteration, m_previousTime is not set
            Time dt = (m_previousTime == std::nullopt) ? 0_msec : now - *m_previousTime;
            m_previousTime = now;
            
            // compute and scale derivative to delta time
            const Number derivative = (dt != 0_sec) ? (m_error - m_previousError) / to_sec(dt) : 0;

            // calculate dt-scaled integral
            m_integral += m_error * to_sec(dt);
            // if the sign of error changed, wipe integral
            if (units::sgn(m_error) != units::sgn((m_previousError)) && m_signFlipReset) m_integral = 0;
            // apply anti windup range
            if (abs(m_error) > m_windupRange && m_windupRange != 0) m_integral = 0;

            m_previousError = m_error;

            // output
            return m_error * m_gains.kP + m_integral * m_gains.kI + derivative * m_gains.kD;
        }

        /**
         * @brief Resets the integral and derivative values of the PID controller.
         */
        void reset() {
            m_previousError = 0;
            m_integral = 0;
        }

        /**
         * @brief Change whether the integral is reset when the error changes sign
         *
         * @param signFlipReset whether to reset the integral when the error changes sign
         */
        void setSignFlipReset(bool signFlipReset) { m_signFlipReset = signFlipReset; }

        /**
         * @brief Get the sign flip reset value
         *
         * @return true
         * @return false
         */
        bool getSignFlipReset() { return m_signFlipReset; }

        /**
         * @brief Set the windup range
         *
         * @param windupRange the new windup range
         */
        void setWindupRange(const Number windupRange)  { m_windupRange = windupRange; }

        /**
         * @brief Get the windup range
         *
         * @return Number
         */
        Number getWindupRange() { return m_windupRange; }

    private:
        Gains m_gains;
        Number m_target = 0;
        Number m_error = 0;

        bool m_signFlipReset;
        Number m_windupRange;

        Number m_previousError = 0;
        Number m_integral = 0;

        std::optional<Time> m_previousTime = std::nullopt;
};