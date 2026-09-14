#include "motions/ramseteTrajectoryFollower.hpp"
// #include "LemLog/logger/Helper.hpp"
#include "motion-handler/MotionCancelHelper.hpp"
#include "Timer.hpp"
#include "util.hpp"

using namespace units;

// static logger::Helper logHelper("atomic/motions/followTrajectory");

void followTrajectory(const Trajectory& t, RameseteFollowSettings settings) {
    std::vector<Trajectory::State> states = t.states();
    
    if (states.size() == 0) {
         // logHelper.error("No were detected in trajectory. Skipping motion");
         return;
    }
    
    MotionCancelHelper helper(10_msec); // cancel helper

    // this value will let us convert linear velocity to motor voltage
    const auto max_velocity = toLinear<AngularVelocity>(max_rpm, wheel_diameter);

    for(int i = 1; i < states.size(); ++i) {
        if(helper.wait()) {
            // get the current position of the robot
            const units::Pose curState = units::Pose(
                settings.poseGetter().x,
                settings.poseGetter().y,
                settings.poseGetter().orientation
            );

            DriveVelocities driveVel = settings.ramseteController.calculate(curState, states[i]);

            // side velocity = linear velocity +/- angular velocity * track_width/2
            const auto leftSideVelocity = driveVel.v - toLinear<AngularVelocity>(driveVel.omega, track_width);
            const auto rightSideVelocity = driveVel.v + toLinear<AngularVelocity>(driveVel.omega, track_width);

            left_motors.move(leftSideVelocity / max_velocity);
            right_motors.move(rightSideVelocity / max_velocity);
            
            // time to wait (msec) = delta distance / output velocity
            Time timeToWait = states[i].pose.distanceTo(states[i - 1].pose) / driveVel.v;

            pros::delay(timeToWait.convert(msec));
        }else{
            break;
        }
    }

    // stop the robot
    left_motors.brake();
    right_motors.brake();
}