// common/hal/src/cosys_fc_link.cpp
// Cosys-AirSim flight controller link compilation unit.
// Entirely guarded by HAVE_COSYS_AIRSIM — compiles to nothing without the SDK.
//
// All methods are defined inline in cosys_fc_link.h (header-only pattern
// matching the other Cosys HAL backends). This .cpp provides a compilation
// unit for the AirSim SDK linkage and ensures the header is compilable.
//
// Issue: #490
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_fc_link.h"

#endif  // HAVE_COSYS_AIRSIM
