#pragma once

#include <cassert>
#include <unordered_map>    
// note: 
// 'unordered_map' is an associative container that stores "key-value" pairs with unique keys using hashing.
//  It is a fast method of storing and retriving unordered data.
//   - provides average O(1) time complexity for search, insertion, and deletion

#include "command.h"
#include "subsystem.h"
#include "eventLoop.h"
#include "pros/misc.hpp"

/**
 * @brief The \refitem CommandScheduler is a singleton class that manages subsystem commands,
 * simplifies the design of complex actions, and helps cut down on task conflicts.
 * 
 * @note This current implementation is directly inspired by WPILib and Echo's old codebase
 */
class CommandScheduler {
    private:
        std::unordered_map<Subsystem*, Command*> subsystems;
        std::unordered_map<Subsystem*, Command*> requirements;
        std::vector<Command*> scheduledCommands;

        // teleop tasks only run if the robot is in driver control
        EventLoop teleopEventLoop{};
        EventLoop eventLoop{};

        bool inRunLoop = false;

        std::vector<Command*> toSchedule;
        std::vector<Command*> toCancel;

        CommandScheduler() = default;
    public:
        // Singleton pattern
        static CommandScheduler& getInstance() {
            static CommandScheduler instance;
            return instance;
        }

        static void registerSubsystem(Subsystem* subsystem, Command* default_command) {
            CommandScheduler& instance = getInstance();     // singleton

            // Make sure the subsystem isn't already registered
            assert(!instance.subsystems.contains(subsystem));

            // Make sure the default command isn't null
            assert(default_command != nullptr);

            instance.subsystems[subsystem] = default_command;
        }

        static void schedule(Command* command) {
	    	CommandScheduler& instance = getInstance();     // singleton

            // Return if the command is already scheduled
            if (command == nullptr || scheduled(command)) {
                return;
            }

            if (instance.inRunLoop) {
                instance.toSchedule.emplace_back(command);
            }

            // return if competition is disabled
            if (pros::competition::is_disabled()) {
                return;
            }

            std::vector<Command*> intersection;

            bool all_interruptible = true;

            auto requirements = command->getRequirements();

            for (auto requirement : instance.requirements) {
                if (find(requirements.begin(), requirements.end(), requirement.first) != requirements.end()) {
                    all_interruptible &= requirement.second->getCancelBehavior() == CommandCancelBehavior::CancelRunning;
                    intersection.push_back(requirement.second);
                }	    
            }

            if (all_interruptible) {
                for (auto intersect : intersection) {
                    intersect->end(true);
                    std::erase(instance.scheduledCommands, intersect);		
                }

                for (auto requirement : requirements) {
                    instance.requirements[requirement] = command;		
                }
		
                command->initialize();

                instance.scheduledCommands.push_back(command);
            }
        }

        static std::optional<Command*> getRequiring(Subsystem* subsystem) {
            CommandScheduler& instance = getInstance();     // singleton

            // return requirements if subsystem is not at end condition
            if (instance.requirements.find(subsystem) != instance.requirements.end()) {
                return instance.requirements[subsystem];
            }

            // if subsystem is at end condition, return null
            return std::nullopt;
        }

        static void run() {
            CommandScheduler& instance = getInstance();     //singleton

            // Run the periodic for all registered subystems
            for (const auto& pair : instance.subsystems) {
                pair.first->periodic();
            }

            // Poll user set event loops
            instance.eventLoop.poll(); 

            // Only poll teleop tasks when the robot controller is active
            if (!pros::competition::is_autonomous() && ! pros::competition::is_disabled()) {
                instance.teleopEventLoop.poll(); 
            }

            instance.inRunLoop = true; 

            // exectute commands and then remove ended commands
            for (auto command : instance.scheduledCommands) {
                command->execute();

                if (command->isFinished()) {
                    command->end(false);

                    for (auto requirement : command->getRequirements()) {
                        instance.requirements.erase(requirement);
                    }

                    std::erase(instance.scheduledCommands, command); 
                }
            }

            instance.inRunLoop = false; 

            // cancel and schedule commands
            for (const auto command : instance.toCancel) {
                cancel(command);
            }
            for (const auto command : instance.toSchedule) {
                schedule(command);
            }

            // erase toCancel and toSchedule for the next cycle
            instance.toCancel.clear();
            instance.toSchedule.clear();

            // schedule subsystem commands if subsystem is not in use
            for (auto [subsystem, command] : instance.subsystems) {
                if (!instance.requirements.contains(subsystem)) {
                    schedule(command);
                }
            }
        }

        static bool scheduled(const Command* command) {	
            CommandScheduler& instance = getInstance();     // singleton
    
            return std::find(instance.scheduledCommands.begin(), instance.scheduledCommands.end(), command) != instance.
            scheduledCommands.end();
        }

        static EventLoop* getEventLoop() {
            CommandScheduler& instance = getInstance();     // singleton

            return &instance.eventLoop;
        }

        static EventLoop* getTeleopEventLoop() {
            CommandScheduler& instance = getInstance();     // singleton

            return &instance.teleopEventLoop;
        }

        static void cancel(Command* command) {
            CommandScheduler& instance = getInstance();     // singleton

            // if in run loop, add command to cancel list
            if (instance.inRunLoop) {
                instance.toCancel.emplace_back(command);
            }

            // if the command is scheduled, remove it from schedule, and free up it's requirements
            if (scheduled(command)) {
                command->end(true);

                std::erase(instance.scheduledCommands, command); 

                for (auto requirement : command->getRequirements()) {
                    instance.requirements.erase(requirement);
                }
            }
        }
};

inline void Command::schedule() {
	CommandScheduler::schedule(this);
}

inline void Command::cancel() {
	CommandScheduler::cancel(this);
}

inline bool Command::scheduled() const {
	return CommandScheduler::scheduled(this);
}
