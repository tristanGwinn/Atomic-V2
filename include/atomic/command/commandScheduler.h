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

        // EventLoop teleopEventLoop{};		// not likely needed
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
            CommandScheduler& instance = getInstance();

            // Make sure the subsystem isn't already registered
            assert(!instance.subsystems.contains(subsystem));

            // Make sure the default command isn't null
            assert(default_command != nullptr);

            instance.subsystems[subsystem] = default_command;
        }

        static void schedule(Command* command) {
	    	CommandScheduler& instance = getInstance();

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

        // static std::optional<Command*> getRequiring(Subsystem* subsystem) {...

        // static void run() {...

        static bool scheduled(const Command* command) {	
            CommandScheduler& instance = getInstance();
    
            return std::find(instance.scheduledCommands.begin(), instance.scheduledCommands.end(), command) != instance.
            scheduledCommands.end();
        }

        // TODO:

};