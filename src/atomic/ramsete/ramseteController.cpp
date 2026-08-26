#pragma once

#include "atomic/ramsete/ramseteController.hpp"
#include "atomic/util.hpp"

#include <cmath>
#include <math.h>

using namespace atomic;

RamseteController::RamseteController(double b, double zeta)
    : m_b{b}, m_zeta{zeta} {}

bool RamseteController::atReference() const {
    const auto& error = m_poseError;
    const auto& tolerance = m_poseTolerance;
    return std::fabs(error.x < tolerance.x) &&
        std::fabs(error.y < tolerance.y) &&
        std::fabs(deg_to_rad(error.theta)) < deg_to_rad(tolerance.theta);
}

void RamseteController::setTolerance(const Pose& poseTolerance) {
    m_poseTolerance = poseTolerance;
}

DriveVelocities RamseteController::calculate(
        const Pose& currentPose, const Pose& poseRef,
        LinearVelocity linearVelocityRef,
        AngularVelocity angularVelocityRef) {
    if (!m_enabled) {
        return DriveVelocities{linearVelocityRef, angularVelocityRef};
    }

    m_poseError = Pose(poseRef.x - currentPose.x,
                       poseRef.y - currentPose.y,
                       poseRef.theta - currentPose.theta);

    double eX = m_poseError.x;
    double eY = m_poseError.y;
    double eTheta = deg_to_rad(m_poseError.theta);
    double vRef = linearVelocityRef.convert(mps);
    double omegaRef = angularVelocityRef.convert(radps);

    double k =
        2.0 * m_zeta * std::sqrt(std::pow(omegaRef, 2) + m_b * std::pow(vRef, 2));

    LinearVelocity v = (vRef * cos(m_poseError.theta) + k * eX) * mps;
    AngularVelocity omega = (omegaRef + k * eTheta +
                             m_b * vRef * sinc(eTheta) * eY) * radps;
    return DriveVelocities{v, omega};
}

DriveVelocities RamseteController::calculate(
        const Pose& currentPose, const Trajectory::State& desiredState) {
    return calculate(currentPose, desiredState.pose,
                     desiredState.linearVelocity, desiredState.angularVelocity);
}

void RamseteController::setEnabled(bool enabled) {
    m_enabled = enabled;
}
