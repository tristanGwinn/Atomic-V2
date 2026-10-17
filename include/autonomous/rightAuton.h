#pragma once

#include "autonomous/commonCommands.h"
#include "subsystems/subsystems.h"
#include "commands/ramsete.h"

/**
 * Skills autonomous
 */
class Right {
public:
    /**
     * Build skills command
     *
     * @return Command that runs skills
     */
    static Command *right() {
        return new InstantCommand([]() { std::cout << "TODO: create right-side autonomous routine." << std::endl; }, {});
    }
};
