// common/hal/src/cosys_imu.cpp
// Cosys-AirSim IMU backend compilation unit.
// Entirely guarded by HAVE_COSYS_AIRSIM — compiles to nothing without the SDK.
//
// All methods are defined inline in cosys_imu.h (header-only pattern matching
// other HAL backends like GazeboIMUBackend). This .cpp provides a compilation
// unit for the AirSim SDK linkage and ensures the header is compilable.
//
// Issue: #434, #462
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_imu.h"

#endif  // HAVE_COSYS_AIRSIM
