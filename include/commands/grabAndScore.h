#pragma once

#include <utility>

#include "command/command.h"
#include "command/commandScheduler.h"
#include "command/waitCommand.h"
#include "command/parallelCommandGroup.h"
#include "command/sequence.h"

#include "subsystems/lift.h"
#include "subsystems/arm.h"
#include "subsystems/claw.h"

/**
 * this command will wait until the claw detects
 * an object, proceed to grab it, and then move the
 * arm (chain bar) and cascade lift to the desired position
 */
class GrabAndScore : public Command {
    private:
        // subsystems
        LiftSubsystem *lift;
        ArmSubsystem *arm;
        ClawSubsystem *claw;

        ParallelCommandGroup *move_to_score_command = nullptr;
        bool inScoringPosition = false;
        
        // Score positions for the arm and lift
        // angle, height
        std::array<double, 2> score_positions;  // it might be better to make this a struct or some other data type

        // maybe a bool or enum to determine order of arm and lift movement
        // may also want to motion profile or smth; problem for future me
    public:
        GrabAndScore(LiftSubsystem *lift, ArmSubsystem *arm, ClawSubsystem *claw, std::array<double, 2> positions)
        : lift(lift), arm(arm), claw(claw), score_positions(std::move(positions)) {}

        void initialize() override {
            arm->holdPositionCommand()->schedule();
            lift->holdPositionCommand()->schedule();
            claw->levelCommand(false)->schedule();

            move_to_score_command =
                new ParallelCommandGroup({
                    lift->positionCommand(from_in(score_positions[1]))
                        ->andThen(lift->holdPositionCommand()),
                    arm->positionCommand(score_positions[0])
                        ->andThen(arm->holdPositionCommand())
                });
        }

        // before writing this, move to position functionality to the cascade lift subsystem
        void execute() override {
            if (claw->getCupStatus() && !claw->getClampStatus() && !inScoringPosition) {
                printf("Detected cup and clamped!\n");
                claw->levelCommand(true)->schedule();
            }
            else if (claw->getClampStatus() && !inScoringPosition) {
                printf("told the lift and arm to move to scoring position ...\n");
                move_to_score_command->schedule();
                if (move_to_score_command->isFinished()) inScoringPosition = true;
            } 
            else if (inScoringPosition && claw->getClampStatus()) {
                CommandScheduler::schedule(
                    claw->levelCommand(false)
                    ->andThen(lift->positionCommand(lift->getPosition() + 4.5_cm))
                    ->andThen(
                        new ParallelCommandGroup({
                            lift->positionCommand(0_m)
                                ->andThen(lift->dropLiftCommand()),
                            arm->positionCommand(0)
                                ->andThen(arm->pctCommand(-0.5))
                                ->withTimeout(1.0_sec)
                        })
                    )
                );
                printf("Send commands to drop the cup and reset position.\n");
            } 
            else {
                printf("waiting to clamp ...\n");
            }
        }

        bool isFinished() override {}

        void end(bool interupted) override {}

        std::vector<Subsystem *> getRequirements() override { return {lift, arm, claw}; }

        ~GrabAndScore() override = default;
};
