#pragma once

#include "autonomous/commonCommands.h"
#include "subsystems/subsystems.h"
#include "commands/ramsete.h"

/**
 * Essential autonomous: just move the robot forward about 14 inches
 */
class Essential {
public:
    /**
     * Build essential command
     *
     * @return Command that runs the essential autonomous routine
     */
    static Command *essential() {
        auto path = CommonCommands::profiledPath({0_in, 0_in}, {2_in, 0.1_in}, {10_in, -0.1_in}, {14_in, 0_in});
        return new Ramsete(drivetrain, path, config::ramsete_tolerance, false);
    }
};
