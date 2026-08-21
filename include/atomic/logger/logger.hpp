#pragma once

#include <memory>
#include <array>

#define FMT_HEADER_ONLY
#include "fmt/core.h"

#include "atomic/logger/baseSink.hpp"
#include "atomic/logger/infoSink.hpp"
#include "atomic/logger/telemetrySink.hpp"

// This was ripped directly from LemLib
// The namespaced was changed from lemlib to atomic for consistency

namespace atomic {

/**
 * @brief Get the info sink.
 * @return std::shared_ptr<InfoSink>
 */
std::shared_ptr<InfoSink> infoSink();

/**
 * @brief Get the telemetry sink.
 * @return std::shared_ptr<TelemetrySink>
 */
std::shared_ptr<TelemetrySink> telemetrySink();
} // namespace atomic
