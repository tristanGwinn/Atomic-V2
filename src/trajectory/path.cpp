#include "trajectory/path.hpp"

CubicBezier::CubicBezier(units::V2Position p0, units::V2Position p1,
                         units::V2Position p2, units::V2Position p3) {
    this->points << p0.x.internal(), p0.y.internal(), 
                    p1.x.internal(), p1.y.internal(), 
                    p2.x.internal(), p2.y.internal(), 
                    p3.x.internal(), p3.y.internal();

    this->matCoefficients << -1, 3, -3, 1, 3, -6, 3, 0, -3, 3, 0, 0, 1, 0, 0, 0;
    this->derivativeCoefficients << -3, 9, -9, 3, 6, -12, 6, 0, -3, 3, 0, 0;
    this->secondDerivativeCoefficients << -6, 18, -18, 6, 6, -12, 6, 0;
}

units::V2Position CubicBezier::getPoint(Time t) {
    auto t_num = t.internal();
    Eigen::Matrix<double, 1, 4> T;
    T << t_num * t_num * t_num,
         t_num * t_num, 
         t_num, 
         1;

    auto result = (T * this->matCoefficients) * this->points;

    return {result(0) * m, result(1) * m};
}

units::V2Velocity CubicBezier::getDerivative(Time t) {
    auto t_num = t.internal();
    Eigen::Matrix<double, 1, 3> T;
    T << t_num * t_num, t_num, 1;

    Eigen::Matrix<double, 1, 2> result =
        T * this->derivativeCoefficients * this->points;

    return {result(0) * mps, result(1) * mps};
}

units::V2Acceleration CubicBezier::getSecondDerivative(Time t) {
    auto t_num = t.internal();
    Eigen::Matrix<double, 1, 2> T;
    T << t_num, 1;

    Eigen::Matrix<double, 1, 2> result =
        T * this->secondDerivativeCoefficients * this->points;

    return {result(0) * mps2, result(1) * mps2};
}

Time CubicBezier::GetMaxT() const { return 1.0_sec; }

units::V2Position MultiPath::getPoint(Time t) {
    Time totalT = 0_sec;
    for (auto path : paths) {
        if (t <= totalT + path->GetMaxT()) {
            return path->getPoint(t - totalT);
        }
        totalT += path->GetMaxT();
    }

    return paths.back()->getPoint(paths.back()->GetMaxT());
}

units::V2Velocity MultiPath::getDerivative(Time t) {
    Time totalT = 0_sec;
    for (auto path : paths) {
        if (t <= totalT + path->GetMaxT()) {
            return path->getDerivative(t - totalT);
        }
        totalT += path->GetMaxT();
    }

  return paths.back()->getDerivative(paths.back()->GetMaxT());
}

units::V2Acceleration MultiPath::getSecondDerivative(Time t) {
    Time totalT = 0_sec;
    for (auto path : paths) {
        if (t <= totalT + path->GetMaxT()) {
            return path->getSecondDerivative(t - totalT);
        }
        totalT += path->GetMaxT();
    }
    
    return paths.back()->getSecondDerivative(paths.back()->GetMaxT());
}

Time MultiPath::GetMaxT() const {
    Time totalT = 0_sec;
    for (auto path : paths) {
        totalT += path->GetMaxT();
    }
    
    return totalT;
}