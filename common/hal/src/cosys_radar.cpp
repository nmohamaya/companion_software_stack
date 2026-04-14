// common/hal/src/cosys_radar.cpp
// Cosys-AirSim radar backend implementation.
// Entirely guarded by HAVE_COSYS_AIRSIM — compiles to nothing without the SDK.
// Issue: #434
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_radar.h"

// Header-only implementation — all methods are defined inline in cosys_radar.h.
// This .cpp exists to provide a compilation unit for the AirSim SDK linkage
// when the real RPC calls are added (TODO #434).
//
// Future: #include <airsim/RpcLibClientBase.hpp>
//         Implement radar polling thread, RPC connection management, etc.

#endif  // HAVE_COSYS_AIRSIM
