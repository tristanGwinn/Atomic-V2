#include "atomic/ramsete/ramseteController.hpp"
#include "atomic/util.hpp"

#include <cmath>
#include <math.h>

namespace atomic {

RamseteController::RamseteController(Number b, Number zeta)
    : m_b{b}, m_zeta{zeta} {}

bool RamseteController::atReference() const {
    const auto& error = m_poseError;
    const auto& tolerance = m_poseTolerance;
    return std::fabs(error.x < tolerance.x) &&
        std::fabs(error.y < tolerance.y) &&
        std::fabs(error.orientation.convert(rad)) < tolerance.orientation.convert(rad);
}

void RamseteController::setTolerance(const units::Pose& poseTolerance) {
    m_poseTolerance = poseTolerance;
}

DriveVelocities RamseteController::calculate(
        const units::Pose& currentPose, const units::Pose& poseRef,
        LinearVelocity linearVelocityRef,
        AngularVelocity angularVelocityRef) {
    if (!m_enabled) {
        return DriveVelocities{linearVelocityRef, angularVelocityRef};
    }

    m_poseError = units::Pose(poseRef.x - currentPose.x,
                       poseRef.y - currentPose.y,
                       poseRef.orientation - currentPose.orientation);

    Length eX = m_poseError.x;
    Length eY = m_poseError.y;
    Angle eTheta = m_poseError.orientation;
    LinearVelocity vRef = linearVelocityRef;
    AngularVelocity omegaRef = angularVelocityRef;

    Number k =
        2.0 * m_zeta * units::sqrt((omegaRef * omegaRef).internal() + m_b * (vRef * vRef).internal());

    LinearVelocity v = mps * (vRef.internal() * units::cos(m_poseError.orientation) + k * eX.internal());
    AngularVelocity omega = radps * (omegaRef.internal() + k * eTheta.internal() +
                                     m_b * vRef.internal() * atomic::sinc(eTheta.internal()) * eY.internal());
    return DriveVelocities{v, omega};
}

DriveVelocities RamseteController::calculate(
        const units::Pose& currentPose, const Trajectory::State& desiredState) {
    return calculate(currentPose, desiredState.pose, desiredState.linearVelocity, desiredState.angularVelocity);
}

void RamseteController::setEnabled(bool enabled) {
    m_enabled = enabled;
}


}   // namespace atomic