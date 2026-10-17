#pragma once

#include "autonConfig.h"
#include "autonomous/autonTypes.h"

#include "command/command.h"
#include "command/instantCommand.h"

#include "autonomous/skillsAuton.h"
#include "autonomous/rightAuton.h"
#include "autonomous/leftAuton.h"
#include "autonomous/essentialAuton.h"

/**
 * Allows easy selection of autonomous routines given a AUTON object, called on initialization to build states
 */
class AutonomousCommands {
public:
    static Command *getAuton() {
        switch (AUTON) {
            case SKILLS:
                return Skills::skills();
            case LEFT:
                return Left::left();
            case RIGHT:
                return Right::right();
            case ESSENTIAL:
                 return Essential::essential();
            default:
                return new InstantCommand([]() { std::cout << "No auton" << std::endl; }, {});
        }
    }
};