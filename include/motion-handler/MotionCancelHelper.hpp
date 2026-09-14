#pragma once

#include "units/units.hpp"



/**
 * @class MotionCancelHelper
 * @brief This class exists to simplify the cancellation logic in a motion function.
 * @note Like MotionHandler, this is borrowed from lemlib
 */
class MotionCancelHelper {
    public:

        /**
         * @brief Construct a new Motion Cancel Helper object
         *
         * @param period how often to update
         */
        MotionCancelHelper(Time period);

        /**
         * @brief wait a certain amount of time
         *
         * This function will return true normally. However, if the task has been notified
         * (the motion handler requests the motion to end), or if the competition state changes,
         * the task will return false, indicating that the motion should end.
         *
         * @returns true if the motion should continue, false otherwise
         */
        bool wait();

        /**
         * @brief Get the amount of time between the current iteration and the last iteration
         *
         * @return Time the time between the current iteration and the last iteration
         */
        Time getDelta();
        
    private:
        bool m_firstIteration = true;
        std::uint32_t m_prevTime;
        Time m_prevPrevTime = 0_msec;
        const int m_originalCompStatus;
        const Time m_period;
};