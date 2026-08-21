#pragma once

#include "atomic/logger/message.hpp"
#include "atomic/logger/baseSink.hpp"

// This was ripped directly from LemLib
// The namespaced was changed from lemlib to atomic for consistency

namespace atomic {
/**
 * @brief Sink for sending messages to the terminal.
 *
 * This is the primary way of interacting with LemLib's logging implementation. This sink is used for printing useful
 * information to the user's terminal.
 * <h3> Example Usage </h3>
 * @code
 * lemlib::infoSink()->setLowestLevel(lemlib::Level::INFO);
 * lemlib::infoSink()->info("info: {}!", "my cool info here");
 * // Specify the order or placeholders
 * lemlib::infoSink()->debug("{1} {0}!","world", "hello");
 * // Specify the precision of floating point numbers
 * lemlib::infoSink()->warn("Thingy exceeded value: {:.2f}!", 93.1234);
 * @endcode
 */
class InfoSink : public BaseSink {
    public:
        /**
         * @brief Construct a new Info Sink object
         */
        InfoSink();
    private:
        /**
         * @brief Log the given message
         *
         * @param message
         */
        void sendMessage(const Message& message) override;
};
} // namespace atomic
