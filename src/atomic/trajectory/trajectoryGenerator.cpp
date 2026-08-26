#include "atomic/trajectory/trajectoryGenerator.hpp"
#include "units/Pose.hpp"
#include "units/Angle.hpp"

#include <iostream>


using namespace atomic;

void TrajectoryGenerator::generateTrajectory(Path *path) {
    Time t = 0_sec;

    trajectoryStates.clear();
    trajectoryStates.push_back(Trajectory::State(
            0_sec, 0_mps, 0_rps,
            units::Pose(path->getPoint(0_sec).x, path->getPoint(0_sec).y, 
                units::atan2(path->getDerivative(0_sec).y, path->getDerivative(0_sec).x)
            )   
        )    
    );

    Trajectory::State lastState = trajectoryStates.back();

    // TODO: edit referenced classes to handle t as a time value
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
            t, maxSpeed,
            toAngularVelocity<LinearVelocity>(maxSpeed, curvature),
            units::Pose(path->getPoint(t).x, path->getPoint(t).y, 
                units::atan2(path->getDerivative(t).y, path->getDerivative(t).x)
            )   
        );

        // delta Time = delta Distance / Velocity
        Time dt = m_deltaD / units::sqrt(derivative.x * derivative.x + derivative.y * derivative.y);

        trajectoryStates.push_back(lastState);

        t += dt;
    }

    int i = trajectoryStates.size() - 1;
    t = path->GetMaxT();
    lastState = Trajectory::State(
        0_sec, 0_mps, 0_rps, 
        units::Pose(path->getPoint(t).x, path->getPoint(t).y, 
            units::atan2(path->getDerivative(t).y, path->getDerivative(t).x)
        )   
    );

    while (t > 0_sec) {
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
            t, maxSpeed,
            toAngularVelocity<LinearVelocity>(maxSpeed, curvature),
            units::Pose(path->getPoint(t).x, path->getPoint(t).y, 
                units::atan2(path->getDerivative(t).y, path->getDerivative(t).x)
            )   
        );

        Time dt = m_deltaD / units::sqrt(derivative.x * derivative.x + derivative.y * derivative.y);

        t -= dt;
        i--;
  }

}

std::vector<Trajectory::State> *TrajectoryGenerator::getTrajectory() { return &trajectoryStates; }