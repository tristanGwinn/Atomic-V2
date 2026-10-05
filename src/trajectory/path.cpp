#include "trajectory/path.hpp"
#include "config.hpp"

CubicBezier::CubicBezier(units::V2Position p0, units::V2Position p1,
                         units::V2Position p2, units::V2Position p3) {
    this->points << p0.x.internal(), p0.y.internal(), 
                    p1.x.internal(), p1.y.internal(), 
                    p2.x.internal(), p2.y.internal(), 
                    p3.x.internal(), p3.y.internal();

    this->matCoefficients << -1, 3, -3, 1, 3, -6, 3, 0, -3, 3, 0, 0, 1, 0, 0, 0;
    this->derivativeCoefficients << -3, 9, -9, 3, 6, -12, 6, 0, -3, 3, 0, 0;
    this->secondDerivativeCoefficients << -6, 18, -18, 6, 6, -12, 6, 0;

    // compute path approximate length using discrete samples
    double t = 0;
    double prev_t = 0;
    while(t < 1) {
        t += 1.0 / config::path_resolution;

        this->path_length
            += units::sqrt( 
                units::square(this->getPoint(t).x - this->getPoint(prev_t).x) +
                units::square(this->getPoint(t).y - this->getPoint(prev_t).y)
            );

        prev_t = t;
    }

    std::cout << "New path length (in meters) is " << this->path_length.internal() << std::endl;
}

units::V2Position CubicBezier::getPoint(double t) {
    Eigen::Matrix<double, 1, 4> T;
    T << t * t * t,
         t * t, 
         t, 
         1;

    auto result = (T * this->matCoefficients) * this->points;

    return {result(0) * m, result(1) * m};
}

units::V2Velocity CubicBezier::getDerivative(double t) {
    Eigen::Matrix<double, 1, 3> T;
    T << t * t, t, 1;

    Eigen::Matrix<double, 1, 2> result =
        T * this->derivativeCoefficients * this->points;

    return {result(0) * mps, result(1) * mps};
}

units::V2Acceleration CubicBezier::getSecondDerivative(double t) {
    Eigen::Matrix<double, 1, 2> T;
    T << t, 1;

    Eigen::Matrix<double, 1, 2> result =
        T * this->secondDerivativeCoefficients * this->points;

    return {result(0) * mps2, result(1) * mps2};
}

double CubicBezier::GetMaxT() const { return 1.0; }


Length CubicBezier::GetLength() const {
    return this->path_length;
}

units::V2Position MultiPath::getPoint(double t) {
    double totalT = 0;
    for (auto path : paths) {
        if (t <= totalT + path->GetMaxT()) {
            return path->getPoint(t - totalT);
        }
        totalT += path->GetMaxT();
    }

    return paths.back()->getPoint(paths.back()->GetMaxT());
}

units::V2Velocity MultiPath::getDerivative(double t) {
    double totalT = 0;
    for (auto path : paths) {
        if (t <= totalT + path->GetMaxT()) {
            return path->getDerivative(t - totalT);
        }
        totalT += path->GetMaxT();
    }

  return paths.back()->getDerivative(paths.back()->GetMaxT());
}

units::V2Acceleration MultiPath::getSecondDerivative(double t) {
    double totalT = 0;
    for (auto path : paths) {
        if (t <= totalT + path->GetMaxT()) {
            return path->getSecondDerivative(t - totalT);
        }
        totalT += path->GetMaxT();
    }
    
    return paths.back()->getSecondDerivative(paths.back()->GetMaxT());
}

double MultiPath::GetMaxT() const {
    double totalT = 0;
    for (auto path : paths) {
        totalT += path->GetMaxT();
    }
    return totalT;
}

Length MultiPath::GetLength() const {
    Length totalL = 0_m;
    for (auto path : paths) {
        totalL += path->GetLength();
    }
    return totalL;
}
