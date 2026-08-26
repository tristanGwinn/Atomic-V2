#pragma once 

#include "atomic/trajectory/trajectory.h"
#include "atomic/trajectory/kinematics.hpp"
#include "atomic/trajectory/path.hpp"

#include <vector>

namespace atomic{

class TrajectoryGenerator {
    public:
        TrajectoryGenerator(Kinematics *kinematics, Length deltaD)
            : m_kinematics(kinematics), m_deltaD(deltaD) {};

        void generateTrajectory(Path *path);
        std::vector<Trajectory::State>* getTrajectory();

    private:
        Kinematics *m_kinematics;
        Length m_deltaD;
        std::vector<Trajectory::State> trajectoryStates;

};

}   // namespace atomic