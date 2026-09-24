#pragma once

#include "pros/rotation.hpp"

#include "hardware/Encoder/Encoder.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"

#include "units/units.hpp"
#include "units/Angle.hpp"

class TrackingWheel {
    private:
        V5RotationSensor* m_encoder;
        Length m_diameter;
        Length m_offset;
        Number m_ratio;
        Length m_lastTotal;

    public:
        /**
         * @brief Create a new tracking wheel
         *
         * @param port the v5 rotation sensor to use
         * @param diameter the diameter of the wheel
         * @param offset distance between the tracking wheel and the center of rotation in inches
         * @param ratio gear ratio of the tracking wheel, defaults to 1
         */
        TrackingWheel(ReversibleSmartPort port, Length diameter, Length offset, Number ratio = 1)
        : m_encoder(new V5RotationSensor(port)),
          m_diameter(diameter),
          m_offset(offset),
          m_ratio(ratio),
          m_lastTotal(to_stRot(m_encoder->getAngle()) * M_PI * diameter * m_ratio) {}
        
        /**
         * @brief reset the tracking wheel encoder
         *
         * Since the internal encoder object is abstract, it's not known what values errno may be
         * set to in case of a failure.
         *
         * @return INT_MAX an error has occurred, possibly setting errno
         */
        int reset() { return m_encoder->setAngle(0_stDeg); }

        /**
         * @brief Get the distance traveled by the tracking wheel since this function was last called.
         * This function is not thread safe.
         *
         * It is recommended to set the angle of the encoder to 0 before starting to use this function.
         *
         * Since the internal encoder object is abstract, it's not known what values errno may be
         * set to in case of a failure.
         *
         * @return INFINITY an error has occurred, possibly setting errno
         * @return Length the distance the tracking wheel has traveled since the last time
         * the function was called
         */
        Length getDistanceDelta() {
            // calculate delta
            const Length total = this->getDistanceTraveled();
            const Length delta = total - m_lastTotal;
            m_lastTotal = total;
            // return the delta
            return delta;
        }

        /**
         * @brief Get the distance traveled by the tracking wheel
         *
         * @return float distance traveled in inches
         */
        Length getDistanceTraveled() { return to_stRot(m_encoder->getAngle()) * M_PI * m_diameter * m_ratio; }
        
        /**
         * @brief Get the offset of the tracking wheel from the center of rotation
         *
         * @return float offset in inches
         */
        Length getOffset() { return m_offset; }
};