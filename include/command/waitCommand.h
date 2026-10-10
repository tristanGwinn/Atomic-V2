#pragma once

#include "api.h"
#include "command.h"
#include "command/parallelRaceGroup.h"
#include "units/units.hpp"

/**
 * @brief Creates a \refitem Command with no requirements that finishes after a user-specified duration
 */
class WaitCommand : public Command {
	Time startTime = 0_sec;
	Time duration = 0_sec;
public:
	/**
	 * @brief Creates a new WaitCommand that runs for a user-specified duration
	 *
	 * @param duration The duration in QTime to run this \refitem Command
	 */
	explicit WaitCommand(const Time &duration)
		: duration(duration) 
	{
		startTime = from_msec(pros::millis());
	}

	/**
	 * @brief Initializes the WaitCommand and sets the start time of the WaitCommand
	 */
	void initialize() override { std::cout << "Waiting for " << duration << " ..." << std::endl; }

	/**
	 * @brief Returns when the WaitCommand's duration has passed
	 *
	 * @return Returns true if the duration has passed, false otherwise
	 */
	bool isFinished() override {
		std::cout << "DONE WAITING" << std::endl;
		return pros::millis() * msec - startTime > duration;
	}

	~WaitCommand() override = default;
};

inline Command *Command::withTimeout(const Time duration) {
	return new ParallelRaceGroup({new WaitCommand(duration), this});
}

