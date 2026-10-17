#pragma once

#include "autonomous/commonCommands.h"
#include "subsystems/subsystems.h"
#include "commands/ramsete.h"

/**
 * Skills autonomous
 */
class Skills {
public:
    /**
     * Build skills command
     *
     * @return Command that runs skills
     */
    static Command *skills() {
        auto driveForward48 = CommonCommands::profiledPath({0_in, 0_in}, {12_in, 0.1_in}, {32_in, -0.1_in}, {48_in, 0_in});

        return new Sequence({                                       // just making sure ...
            claw->levelCommand(true),
            new Ramsete(drivetrain, driveForward48, config::ramsete_tolerance, false),  new WaitCommand(250_msec),
            (new Rotate(drivetrain, 180_stDeg, false, 0.0, true)),                      new WaitCommand(250_msec),
            drivetrain->pctCommand(-0.6, -0.6),                                         new WaitCommand(15_sec),
            CommonCommands::ScoreAtLevel(1),                                            new WaitCommand(150_msec),
            claw->levelCommand(false),                                                  new WaitCommand(150_msec)
        });
        
        // return new InstantCommand([]() { std::cout << "TODO: create skills autonomous routine." << std::endl; }, {});
    }
};
