#define FMT_HEADER_ONLY
#include "fmt/format.h"
#include "atomic/logger/telemetrySink.hpp"
#include "atomic/logger/stdout.hpp"

namespace atomic {
TelemetrySink::TelemetrySink() { setFormat("TELE_{level}:{message}TELE_END"); }

void TelemetrySink::sendMessage(const Message& message) {
    bufferedStdout().print("\033[s{}\033[u\033[0J", message.message);
}
} // namespace atomic