#include "atomic/motions/ramseteTrajectoryFollower.hpp"
#include "LemLog/logger/Helper.hpp"
#include "atomic/MotionCancelHelper.hpp"
#include "atomic/Timer.hpp"
#include "atomic/util.hpp"

using namespace units;

namespace atomic {

static logger::Helper logHelper("atomic/motions/followTrajectory");

void followTrajectory(const atomic::Trajectory& t, RameseteFollowSettings settings) {
    std::vector<Trajectory::State> states = t.states();
    
    if (states.size() == 0) {
         logHelper.error("No were detected in trajectory. Skipping motion");
         return;
    }
    
    atomic::MotionCancelHelper helper(10_msec); // cancel helper

    // this value will let us convert linear velocity to motor voltage
    // meters per second * velToVoltage -->  new movement voltage
    const auto velToVoltage = (39.37 * 60 * 127) / (M_PI * wheel_diameter.internal() * max_rpm.internal());

    for(int i = 1; i < states.size(); ++i) {
        if(helper.wait()) {
            // get the current position of the robot
            const Pose curState = Pose(
                settings.poseGetter().x.internal(),
                settings.poseGetter().y.internal(),
                settings.poseGetter().orientation.internal()
            );

            DriveVelocities driveVel = settings.ramseteController.calculate(curState, states[i]);

            // side velocity = linear velocity +/- angular velocity * track_width/2
            const auto leftSideVelocity = driveVel.v.internal() - driveVel.omega.internal() * track_width.internal() / 2;
            const auto rightSideVelocity = driveVel.v.internal() + driveVel.omega.internal() * track_width.internal() / 2;

            left_motors.move(velToVoltage * leftSideVelocity);
            right_motors.move(velToVoltage * rightSideVelocity);

            Time timeToWait = states[i].t - states[i - 1].t;
            pros::delay(timeToWait/sec);
        }else{
            break;
        }
    } 

    // stop the robot
    left_motors.brake();
    right_motors.brake();
}

}   // namespace atomic