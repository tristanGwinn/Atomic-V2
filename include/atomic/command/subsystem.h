#pragma once

/**
 * @brief Abstract class for subsystem behaviors
 */
class Subsystem {
    public:
        /**
         * Period is run every frame by the \refitem CommandSchedular; useful for debugging tasks and feedback controllers
         */
        virtual void periodic() = 0;
        virtual ~Subsystem() = default;
};