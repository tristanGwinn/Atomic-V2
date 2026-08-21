#pragma once

// function is based on the VPI adaptation of the WPIlib implementation.
#include "atomic/pose.hpp"
#include "atomic/ramsete/trajectory.h"
#include "units/units.hpp"
#include "units/Angle.hpp"

namespace atomic{

/**
 * This structure type will hold the output velocities
 * 
 * TODO: look into background movement system, and document this.
 * May want to move this somewhere else, depending on how the lemlib movements work
 * 
 */
struct DriveVelocities {
    LinearVelocity v = 0_mps;
    AngularVelocity omega = 0_rps;
};

class RamseteController {
    public:
        /**
         * @brief Ramsete unicycle controller constructor
         * 
         * @param b Tuning parameter (b > 0) for which larger values make
         *          convergence more aggressive like a proportional term.
         * @param zeta Tuning parameter (0 < zeta < 1) for which larger values provide
         *             more damping in response.
         * 
         */
        RamseteController(double b, double zeta);

        /** 
         * @note Construct a Ramsete unicycle controller. The default arguments for
         * b and zeta of 2.0 and 0.7 have been well-tested to produce desirable
         * results for FRC, your milage may vary for VRC
         * 
         * ^^^ Copied from VPI
         */
        RamseteController() : RamseteController(2.0, 0.7) {}

        /**
         * @brief Whether the pose error is within tolerance of the reference
         * 
         */
        bool atReference() const;

        /**
         * @brief Sets the pose tolerance for atReference()
         * 
         * @param poseTolerance Pose error which is tolerable
         * 
         */
        void setTolerance(const Pose& poseTolerance);

        /**
         * @brief Returns the next Controller output
         * 
         * @note The reference pose, linear velocity, and angular velocity should come from
         *       a drivetrain trajectory.
         * 
         * @param currentPose The current pose
         * @param poseRef The desired pose
         * @param linearVelocityRef the desired linear velocity
         * @param angularVelocityRef the desired angular velocity
         * 
         */
        DriveVelocities calculate(const Pose& currentPose, const Pose& poseRef,
                                LinearVelocity linearVelocityRef,
                                AngularVelocity angularVelocityRef);

        /**
         * @brief Returns the next Controller output
         * 
         * @note The reference pose, linear velocity, and angular velocity should come from
         *       a drivetrain trajectory.
         * 
         * @param currentPose The current pose=
         * @param desiredState The desired pose, linear velocity, and angular velocity
         *                     from a trajectory.
         * 
         */
        DriveVelocities calculate(const Pose& currentPose,
                                const Trajectory::State& desiredState);

        /**
         * @brief Enables and disables the controller for troubleshooting purposes.
         *
         * @param enabled If the controller is enabled or not.
         */
        void setEnabled(bool enabled);

    private:
        double m_b;
        double m_zeta;

        Pose m_poseError = Pose(0, 0, 0);
        Pose m_poseTolerance = Pose(0, 0, 0);
        bool m_enabled = true;
};

}   // namespace atomic
