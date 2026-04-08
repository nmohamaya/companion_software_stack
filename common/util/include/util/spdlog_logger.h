// common/util/include/util/spdlog_logger.h
// Convenience header — re-exports SpdlogLogger from ilogger.h.
//
// SpdlogLogger is the default ILogger implementation, defined in
// ilogger.h to make the DRONE_LOG macros self-contained.
// Include this header if you need to explicitly reference SpdlogLogger
// (e.g. to construct one for a non-default spdlog logger instance).
#pragma once

#include "util/ilogger.h"
