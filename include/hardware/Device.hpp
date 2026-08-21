#pragma once

#include <cstdint>

// This was ripped directly from LemLib
// The namespaced was changed from lemlib to atomic for consistency

namespace atomic {
/**
 * @brief abstract Device class
 *
 * used to have a generic interface for devices
 */
class Device {
    public:
        virtual int32_t isConnected() const = 0;
};
} // namespace atomic
