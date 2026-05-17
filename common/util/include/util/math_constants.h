// SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
// Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.
//
// common/util/include/util/math_constants.h
//
// Shared math constants — single source of truth for unit conversions and
// commonly-used numerical constants reused across HAL, planner, and tests.
//
// Origin: extracted in PR #763 (Epic #740 Layer 4) after `kRadToDeg` use
// appeared in both `common/hal/include/hal/cosys_radar.h` (PR #636) and
// `process4_mission_planner/include/planner/mission_state_tick.h` (Layer 4
// settle gate, this PR).  Centralising avoids the literal `57.2957795f`
// drift across modules and keeps the conversion factor one source-edit
// away.
//
// Header-only by design — `inline constexpr` so multiple translation units
// can include without ODR concerns.

#pragma once

namespace drone::util {

inline constexpr float kPi     = 3.14159265358979323846f;
inline constexpr float kTwoPi  = 2.0f * kPi;
inline constexpr float kHalfPi = 0.5f * kPi;

/// Multiply a value in radians by `kRadToDeg` to convert to degrees.
inline constexpr float kRadToDeg = 180.0f / kPi;

/// Multiply a value in degrees by `kDegToRad` to convert to radians.
inline constexpr float kDegToRad = kPi / 180.0f;

}  // namespace drone::util
