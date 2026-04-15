// common/hal/src/cosys_rpc_client.cpp
// Cosys-AirSim shared RPC client implementation.
// Entirely guarded by HAVE_COSYS_AIRSIM — compiles to nothing without the SDK.
// Issue: #461
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_rpc_client.h"

// Header-only implementation — all methods are defined inline in cosys_rpc_client.h.
// This .cpp exists to provide a compilation unit for the AirSim SDK linkage
// when the real RPC calls are added (TODO #462).
//
// Future: #include <airsim/RpcLibClientBase.hpp>
//         Move connect()/disconnect()/reconnect() implementations here once
//         they depend on the AirSim SDK types.

#endif  // HAVE_COSYS_AIRSIM
