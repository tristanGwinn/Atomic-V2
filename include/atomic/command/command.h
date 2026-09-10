#pragma once

#include "subsystem.h"
#include "units/units.hpp"
#include <vector>
#include <functional>

/**
 * @brief Enum for different cancel behaviors for Commands
 *
 * Cancel Incoming causes newly scheduled commands to fail, keeping
 * the current command running until it ends uninterrupted. Cancel
 * Running caused the currently running command to yield to the newly
 * scheduled command and end interrupted.
 *
 */
enum class CommandCancelBehavior {
    /**
     * @brief Causes the newly scheduled command(s) to fail upon
     * attempts to schedule them while a command with CancelRunning is
     * reserving the subsystem
     */
    CancelIncoming,
    /**
     * @brief Causes the currently running command to yield to the
     * newly scheduled command upon scheduling.
     */
    CancelRunning,
};

/**
 * @brief Abstract Command class for commands
 *
 * @details Commands are the base of the command based structure, they
 * compartmentalize robot behaviors and provide the structure
 * necessary to ensure that multiple commands are attempting to access
 * the same hardware at the same time. Generally in projects you can
 * use the classes such as \refitem FunctionalCommand, or \refitem
 * RunCommand for simple Command behaviors, and you can override this
 * class to create more complex behaviors - Echo/WPI
 * 
 */
class Command {
    public:
        /**
         * @brief Called before every time the command is used. Users can
         * override this to create starting behaviors for custom commands
         * 
         *  ```C
         *	void initialize() override {
         *      // Initialization tasks here, for example setting motor position
         *	}
         *  ```
         */
        virtual void initialize() {}

        /**
         * @brief Execute is run every frame in the command
         * scheduler(10ms/100Hz or user selected) This can be useful for
         * updating feedback controllers.
         */
        virtual void execute() {}
    
        /**
        * @brief Command can override this function to have custom ending
        * behaviors
        *
        * @return true if the Command is finished, false if it is still
        * running
        */
        virtual bool isFinished() { return false; };

        /**
         * @brief Stops the current command, this can be used to
         * clean up resources or finish up certain behaviors
         * 
         * @param interrupted true if the command was stopped before
         * isFinished returned true, false otherwise
         */
        virtual void end(bool interrupted) {};

        /**
         * @brief This function returns the necessary subsystems needed to
         * do run command. The scheduler will work to free these
         * requirements before running the behavior to ensure no tasks
         * overlap.
         *
         * @warning You must ensure all subsystems neccesary are captured
         * by this function or else multiple commands will overlap and run
         * at the same time
         *
         * @return A vector of all the subsystem requirements for this
         * class
         */
        virtual std::vector<Subsystem *> getRequirements() { return {}; }

        /**
         * @brief Returns the cancel behavior for this class,
         * defaults to CommandCancelBehavior::CancelRunning
         * 
         * @return Desired \refitem CommandCancelBehavior
         */
        virtual CommandCancelBehavior getCancelBehavior() {
            return CommandCancelBehavior::CancelRunning;
        }

        /**
         * 
         * @brief Schedule this command with the \refitem CommandScheduler
         *
         * ```C
         * // Declare command(example)
         * Command* command;
         *
         * // Use shorthand to schedule the command
         * command->schedule();
         *
         * // command is now scheduled
         * ```
         */
        void schedule();

        /**
         * @brief Cancel this command if it is currently scheduled in the
         * \refitem CommandScheduler
         * 
         * @note can be called the same way as schedule();
         * 
         */
        void cancel();

        // TODO:

};