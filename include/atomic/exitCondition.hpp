#pragma once

#include "units/units.hpp"
#include "pros/rtos.hpp"
#include <optional>
#include <vector>

namespace atomic {

template <isQuantity Q> class ExitCondition {
    public:
        /**
         * @brief Create a new Exit Condition
         *
         * @param range the range where the countdown is allowed to start
         * @param time how much time to wait while in range before exiting
         */
        ExitCondition(Q range, Time time)
            : m_range(range),
              m_time(time) {}

        /**
         * @brief update the exit condition
         *
         * @param input the input for the exit condition
         * @return true exit condition met
         * @return false exit condition not met
         */
        bool update(Q input) {
            // if start time is null, assign current time as value
            if (m_startTime == std::nullopt) m_startTime = from_msec(pros::millis());
            
            const Time currentTime = pros::millis() * msec;

            // if out of range, wipe start time
            if (units::abs(input) >= m_range) m_startTime.reset();  
            else if (m_startTime == -1 * sec) m_startTime = currentTime;
            else if (currentTime >= m_startTime.value() + m_time) m_done = true;

            return m_done;
        }

        /**
         * @brief reset the exit condition timer
         */
        void reset(){
            m_startTime.reset();
            m_done = false;
        }
        
    private:
        std::optional<Time> m_startTime = std::nullopt;
        bool m_done = false;
        Q m_range;
        Time m_time;
};

template <isQuantity Q> class ExitConditionGroup {
    public:
        /**
         * @brief Create a new exit condition group
         *
         * @param exitConditions the exit conditions to check
         */
        ExitConditionGroup(std::vector<ExitCondition<Q>> exitConditions)
            : m_exitConditions(exitConditions) {}

        /**
         * @brief Update the exit condition group
         *
         * @param input the input to check
         * @return true
         * @return false
         */
        bool update(Q input) {
            for (auto& exitCondition : m_exitConditions) {
                if (exitCondition.update(input)) return true;
            }
            return false;
        }

        /**
         * @brief Resets the exit condition group
         */
        void reset() {
            for (auto& exitCondition : m_exitConditions) { exitCondition.reset(); }
        }

        /**
         * @brief Set the exit conditions in the group
         *
         * @param exitConditions the new list of exit conditions
         */
        void setExitConditions(std::vector<ExitCondition<Q>> exitConditions) { m_exitConditions = exitConditions; }

        /**
         * @brief Get the exit conditions in the group
         *
         * @return std::vector<ExitCondition> the exit conditions
         */
        std::vector<ExitCondition<Q>> getExitConditions() { return m_exitConditions; }

        /**
         * @brief Add an exit condition to the group
         *
         * @param exitCondition the exit condition to add
         */
        void addExitCondition(ExitCondition<Q> exitCondition) { m_exitConditions.push_back(exitCondition); }
    
        private:
        std::vector<ExitCondition<Q>> m_exitConditions;
};

} // namespace atomic