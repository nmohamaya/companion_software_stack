// common/hal/src/cosys_depth.cpp
// Cosys-AirSim depth estimator backend compilation unit.
// Entirely guarded by HAVE_COSYS_AIRSIM — compiles to nothing without the SDK.
//
// All methods are defined inline in cosys_depth.h (header-only pattern matching
// other HAL backends). This .cpp provides a compilation unit for the AirSim SDK
// linkage and ensures the header is compilable.
//
// Issue: #434, #462
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_depth.h"

#endif  // HAVE_COSYS_AIRSIM
