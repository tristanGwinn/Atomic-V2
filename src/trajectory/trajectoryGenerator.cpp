#include "trajectory/trajectoryGenerator.hpp"
#include "units/Pose.hpp"
#include "units/Angle.hpp"
#include "config.hpp"

#include <iostream>

void TrajectoryGenerator::generateTrajectory(Path *path) {
    printf("Generating Trajectory ...\n");
    double t = 0;   // REMEMBER: t is NOT time, it is a sum of percentages

    trajectoryStates.clear();
    trajectoryStates.push_back(Trajectory::State(
            0_sec, 0_mps, 0_rps,
            units::Pose(path->getPoint(0).x, path->getPoint(0).y, 
                units::atan2(path->getDerivative(0).y, path->getDerivative(0).x)
            )   
        )    
    );

    Trajectory::State lastState = trajectoryStates.back();

    // while the summed percent of each path is less the number of paths, compute the first motion profiling pass
    while(t < path->GetMaxT()) {
        LinearVelocity maxSpeed = m_kinematics->getMaxSpeed(path, lastState, m_deltaD, t);

        auto derivative = path->getDerivative(t);
        auto secondDerivative = path->getSecondDerivative(t);

        // I was having issues getting units::pow(derivative.x * derivative.x + derivative.y * derivative.y, 1.5)
        // to work how I wanted, so this will just have to do
        Curvature curvature =
            (derivative.x * secondDerivative.y - derivative.y * secondDerivative.x) / (
               (derivative.x * derivative.x + derivative.y * derivative.y) 
                * units::sqrt(derivative.x * derivative.x + derivative.y * derivative.y)
            );

        lastState = Trajectory::State(
            t*sec, maxSpeed,
            toAngularVelocity<LinearVelocity>(maxSpeed, curvature),
            units::Pose(path->getPoint(t).x, path->getPoint(t).y, 
                units::atan2(path->getDerivative(t).y, path->getDerivative(t).x) - 90_stDeg
            )   
        );

        // this is an artifact from the original library
        Time dt = m_deltaD / units::sqrt(derivative.x * derivative.x + derivative.y * derivative.y);

        trajectoryStates.push_back(lastState);

        t += dt.internal();
    }

    int i = trajectoryStates.size() - 1;
    t = path->GetMaxT();
    lastState = Trajectory::State(
        0_sec, 0_mps, 0_rps, 
        units::Pose(path->getPoint(t).x, path->getPoint(t).y, 
            units::atan2(path->getDerivative(t).y, path->getDerivative(t).x) - 90_stDeg
        )   
    );

    // iterate back through the trajectory, and compute the second profiling pass
    while (t > 0) {
        trajectoryStates[i] = lastState.linearVelocity < trajectoryStates[i].linearVelocity
                        ? lastState
                        : trajectoryStates[i];

        LinearVelocity maxSpeed = m_kinematics->getMaxSpeed(path, lastState, m_deltaD, t);

        auto derivative = path->getDerivative(t);
        auto secondDerivative = path->getSecondDerivative(t);

        Curvature curvature =
            (derivative.x * secondDerivative.y - derivative.y * secondDerivative.x) / (
               (derivative.x * derivative.x + derivative.y * derivative.y) 
                * units::sqrt(derivative.x * derivative.x + derivative.y * derivative.y)
            );

        lastState = Trajectory::State(
            t*sec, maxSpeed,
            toAngularVelocity<LinearVelocity>(maxSpeed, curvature),
            units::Pose(path->getPoint(t).x, path->getPoint(t).y, 
                units::atan2(path->getDerivative(t).y, path->getDerivative(t).x) - 90_stDeg
            )   
        );

        // this is an artifact from the original library
        Time dt = m_deltaD / units::sqrt(derivative.x * derivative.x + derivative.y * derivative.y);

        t -= dt.internal();
        i--;
    }


    std::cout << "\nTime Parameterizing Trajectory: \n" << std::endl;

    Time path_time = 0_sec;
    auto currentState = trajectoryStates[0];
    lastState = trajectoryStates[0];
    // Time parameterize the path
    for (int i = 0; i <= trajectoryStates.size() - 1; i++) {
        currentState = trajectoryStates[i];

        // v_f^2 = v_0^2 + 2 * a * delta_x → a = ( v_f^2 - v_0^2 ) / ( 2 * delta_x )
        LinearAcceleration segmentAccel = currentState.pose.distanceTo(lastState.pose).internal() == 0 ? 
            0_mps2 :
            ( units::square(currentState.linearVelocity) - units::square(lastState.linearVelocity) ) / 
            ( 2 * currentState.pose.distanceTo(lastState.pose) );

        // delta x = v_0 * t + ½a * t^2 → t = ( -v_0 ± sqrt( v_0^2 + 2 * a * delta_x) ) / a
        // Ms. Korzan would be very proud if she saw this ^^
        LinearVelocity determinate = units::sqrt( units::square(lastState.linearVelocity) + 2 * segmentAccel * currentState.pose.distanceTo(lastState.pose));
        
        path_time += segmentAccel.internal() == 0 ? 0_sec :
                     units::max(-lastState.linearVelocity + determinate, 
                                -lastState.linearVelocity - determinate)
                     / segmentAccel;

        currentState.t = path_time;
        lastState = currentState;

        std::cout << "Delta X: " << currentState.pose.distanceTo(lastState.pose)
                  << "\nState Velocity: " << currentState.linearVelocity
                  << "\nSegment Acceleration: " << segmentAccel
                  << "\nTime: " << currentState.t
                  << "\nPose: ( " 
                    << to_in(currentState.pose.x) << " in, " 
                    << to_in(currentState.pose.y) << "in, " 
                    << to_stDeg(currentState.pose.orientation) << "deg )\n"
                  << std::endl;

        trajectoryStates[i] = currentState;
        trajectoryStates[i].pose.orientation = currentState.pose.orientation;
    }

    std::cout << "\nFinished generating the Trajectory.\n"
              << "Total path time: " << trajectoryStates.back().t << std::endl;
}

std::vector<Trajectory::State> TrajectoryGenerator::getTrajectory() { return trajectoryStates; }