#include "atomic/trajectory/kinematics.hpp"
#include "atomic/util.hpp"

using namespace atomic;

LinearVelocity DifferentialKinematics::getMaxSpeed(Path *path, Trajectory::State lastState, Length deltaD, Time t) const {
    auto derivative = path->getDerivative(t);
    auto secondDerivative = path->getSecondDerivative(t);

    // I was having issues getting units::pow(derivative.x * derivative.x + derivative.y * derivative.y, 1.5)
    // to work how I wanted, so this will just have to do
    Curvature curvature =
        (derivative.x * secondDerivative.y - derivative.y * secondDerivative.x) / (
           (derivative.x * derivative.x + derivative.y * derivative.y) 
            * units::sqrt(derivative.x * derivative.x + derivative.y * derivative.y)
        );
      
    LinearVelocity maxSpeedCurvature = 2 * m_max_vel / (2.0 + m_track_width * abs(curvature));

    LinearVelocity maxSpeedFriction = units::sqrt(m_friction * m_max_accel / abs(curvature));

    LinearVelocity maxSpeedAccel = units::sqrt(lastState.linearVelocity * lastState.linearVelocity
                                               + 2.0 * m_max_accel * deltaD);

    return std::min({maxSpeedCurvature, maxSpeedFriction, maxSpeedAccel, m_max_vel});
}

std::vector<LinearVelocity> DifferentialKinematics::getWheelVelocities(LinearVelocity linearVel, AngularVelocity angularVel) const {
    LinearVelocity leftVel = linearVel - toLinear<AngularVelocity>(angularVel, m_track_width);
    LinearVelocity rightVel = linearVel + toLinear<AngularVelocity>(angularVel, m_track_width);

    return {leftVel, rightVel};
}
