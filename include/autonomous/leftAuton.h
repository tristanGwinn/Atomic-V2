#pragma once

#include "autonomous/commonCommands.h"
#include "subsystems/subsystems.h"
#include "commands/ramsete.h"

/**
 * Skills autonomous
 */
class Left {
public:
    /**
     * Build skills command
     *
     * @return Command that runs skills
     */
    static Command *left() {
        auto driveForward24 = CommonCommands::profiledPath({0_in, 0_in}, {8_in, 0.1_in}, {16_in, -0.1_in}, {24_in, 0_in});
        return new Sequence({
            toggleFlippers->levelCommand(false),
            (new Ramsete(drivetrain, driveForward24, config::ramsete_tolerance, false))->withTimeout(5_sec),
            (drivetrain->pctCommand(-0.4, 0.4))
                ->until([]() { return abs(drivetrain->getPose().orientation - 180_stDeg) < config::angle_tolerance; })->withTimeout(2_sec),
            // (new Rotate(drivetrain, 180_stDeg, false, 0.0, true))->withTimeout(3_sec),
            drivetrain->pctCommand(0, 0),
            (new Ramsete(drivetrain, driveForward24, config::ramsete_tolerance, false)),
            drivetrain->pctCommand(-0.6, -0.6),       new WaitCommand(2_sec),
            drivetrain->pctCommand(0.9, 0.9),         new WaitCommand(2_sec),
            drivetrain->pctCommand(-0.6, -0.6),       new WaitCommand(2_sec),
            drivetrain->pctCommand(0, 0),
            toggleFlippers->levelCommand(true)
        });

        // return new InstantCommand([]() { std::cout << "TODO: create left-side autonomous routine." << std::endl; }, {});
    }
};
