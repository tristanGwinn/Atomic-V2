#pragma once

#include "units/units.hpp"
#include <optional>

namespace atomic {

/**
 * @brief Struct to hold PID gains.
 *
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 */
struct Gains {
        Number kP = 0;
        Number kI = 0;
        Number kD = 0;
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
        PID(Number kP, Number kI, Number kD, Number windupRange = 0, bool signFlipReset = false);

        /**
         * @brief Constructs a new PID controller
         *
         * @param gains the gains to use
         * @param windupRange range at which integral is reset
         * @param signFlipReset whether to reset integral when error changes sign
         */
        PID(const Gains& gains, Number windupRange = 0, bool signFlipReset = false);

        /**
         * @brief Get the current gains
         *
         * @return Gains the current gains
         */
        Gains getGains();

        /**
         * @brief Set the new gains
         *
         * @param gains the new gains
         */
        void setGains(Gains gains);

        /**
         * @brief Updates the PID controller using a given error, and outputs the next control signal.
         *
         * @param error the error from the setpoint. Error is calculated as setpoint - current
         * @return Number the control signal (output)
         */
        Number update(Number error);

        /**
         * @brief Resets the integral and derivative values of the PID controller.
         */
        void reset();

        /**
         * @brief Change whether the integral is reset when the error changes sign
         *
         * @param signFlipReset whether to reset the integral when the error changes sign
         */
        void setSignFlipReset(bool signFlipReset);

        /**
         * @brief Get the sign flip reset value
         *
         * @return true
         * @return false
         */
        bool getSignFlipReset();

        /**
         * @brief Set the windup range
         *
         * @param windupRange the new windup range
         */
        void setWindupRange(Number windupRange);

        /**
         * @brief Get the windup range
         *
         * @return Number
         */
        Number getWindupRange();

    private:
        Gains m_gains;

        bool m_signFlipReset;
        Number m_windupRange;

        Number m_previousError = 0;
        Number m_integral = 0;

        std::optional<Time> m_previousTime = std::nullopt;
};

} // namespace atomic