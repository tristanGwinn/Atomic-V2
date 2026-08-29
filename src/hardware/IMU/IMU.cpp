#include "hardware/IMU/IMU.hpp"
#include <mutex>

namespace atomic {
IMU::IMU(Number scalar)
    : m_gyroScalar(scalar) {}

IMU::IMU(const IMU& other)
    : m_gyroScalar(other.getGyroScalar()) {}

int32_t IMU::setGyroScalar(Number scalar) {
    std::lock_guard lock(m_mutex);
    m_gyroScalar = scalar;
    // Always returns 0 because the scalar setter is not dependent on hardware
    return 0;
}

Number IMU::getGyroScalar() const {
    std::lock_guard lock(m_mutex);
    // Never returns an error because the scalar getter is not dependent on hardware
    return m_gyroScalar;
}
} // namespace atomic