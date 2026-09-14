#pragma once 

#include "trajectory/trajectory.h"
#include "trajectory/kinematics.hpp"
#include "trajectory/path.hpp"

#include <vector>

class TrajectoryGenerator {
    public:
        TrajectoryGenerator(Kinematics *kinematics, Length deltaD)
            : m_kinematics(kinematics), m_deltaD(deltaD) {};

        void generateTrajectory(Path *path);
        std::vector<Trajectory::State> getTrajectory();

    private:
        Kinematics *m_kinematics;
        Length m_deltaD;
        std::vector<Trajectory::State> trajectoryStates;

};