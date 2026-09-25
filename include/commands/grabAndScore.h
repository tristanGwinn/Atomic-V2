#pragma once

#include <utility>

#include "command/command.h"

#include "subsystems/lift.h"
#include "subsystems/arm.h"
#include "subsystems/claw.h"

/**
 * this command will wait until the claw detects
 * an object, proceed to grab it, and then move the
 * arm (chain bar) and cascade lift to the desired position
 * 
 */
class GrabAndScore : public Command {
    private:
        // subsystems
        LiftSubsystem *lift;
        ArmSubsystem *arm;
        ClawSubsystem *claw;
        
        // Score positions for the arm and lift
        std::array<double, 2> score_positions;  // it might be better to make this a struct or some other data type

        // maybe a bool or enum to determine order of arm and lift movement
        // may also want to motion profile or smth; problem for future me
    public:
        GrabAndScore(LiftSubsystem *lift, ArmSubsystem *arm, ClawSubsystem *claw, std::array<double, 2> positions)
        : lift(lift), arm(arm), claw(claw), score_positions(std::move(positions)) {}

        void initialize() override {
            // no-op
        }

        // before writing this, move to position functionality to the cascade lift subsystem
        void execute() override {
            // wait until the claw has detected a cup [ with claw->getCupStatus() ]

            // move the arm and lift (in whatever order we decide)
            // ↑↑↑ this is probably best accomplished using motion profiling, 
            // but currently we do not have applicable functionality for that. Best left to a future iteration ;)
        }

        bool isFinished() override {}

        // this command should end when settled, or when the reset button is pressed / reset command triggered (yet to be written) 
        // ↑↑↑ requires a race command, that header still needs to be written and carried over from the original library
        void end(bool interupted) override {}

        std::vector<Subsystem *> getRequirements() override { return {lift, arm, claw}; }

        ~GrabAndScore() override = default;
};
