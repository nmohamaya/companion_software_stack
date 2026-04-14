// common/hal/src/cosys_camera.cpp
// Cosys-AirSim camera backend implementation.
// Entirely guarded by HAVE_COSYS_AIRSIM — compiles to nothing without the SDK.
// Issue: #434
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_camera.h"

// Header-only implementation — all methods are defined inline in cosys_camera.h.
// This .cpp exists to provide a compilation unit for the AirSim SDK linkage
// when the real RPC calls are added (TODO #434).
//
// Future: #include <airsim/RpcLibClientBase.hpp>
//         Implement image retrieval thread, RPC connection management, etc.

#endif  // HAVE_COSYS_AIRSIM
