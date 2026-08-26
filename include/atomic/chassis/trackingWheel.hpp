#pragma once

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"

#include "units/units.hpp"
#include "units/Angle.hpp"

namespace atomic {

class TrackingWheel {
    public:
        /**
         * @brief Create a new tracking wheel
         *
         * @param encoder the encoder to use for tracking
         * @param diameter the diameter of the wheel
         * @param offset distance between the tracking wheel and the center of rotation in inches
         * @param ratio gear ratio of the tracking wheel, defaults to 1
         */
        TrackingWheel(Encoder* encoder, Length diameter, Length distance, Number ratio = 1);
        
        /**
         * @brief Create a new tracking wheel
         *
         * @param port the v5 rotation sensor to use
         * @param diameter the diameter of the wheel
         * @param offset distance between the tracking wheel and the center of rotation in inches
         * @param ratio gear ratio of the tracking wheel, defaults to 1
         */
        TrackingWheel(ReversibleSmartPort port, Length diameter, Length offset, Number ratio = 1);
        
        /**
         * @brief reset the tracking wheel encoder
         *
         * Since the internal encoder object is abstract, it's not known what values errno may be
         * set to in case of a failure.
         *
         * @return INT_MAX an error has occurred, possibly setting errno
         */
        int reset();

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
        Length getDistanceDelta();

        /**
         * @brief Get the distance traveled by the tracking wheel
         *
         * @return float distance traveled in inches
         */
        Length getDistanceTraveled();

        /**
         * @brief Get the offset of the tracking wheel from the center of rotation
         *
         * @return float offset in inches
         */
        Length getOffset();

    private:
        Encoder* m_encoder;
        Length m_diameter;
        Length m_offset;
        Number m_ratio;
        Length m_lastTotal;
};

}   // atomic