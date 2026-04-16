# ============================================================================
# FindAirSim.cmake — Locate and build vendored Cosys-AirSim C++ SDK
# ============================================================================
# Uses ExternalProject_Add to build AirLib + rpclib + MavLinkCom from the
# vendored git submodule at third_party/cosys-airsim/. ExternalProject builds
# in a separate CMake invocation, which isolates AirSim's aggressive global
# flag mutations (CommonSetup.cmake adds -Wno-unused, etc.) from our build.
#
# Outputs:
#   COSYS_AIRSIM_FOUND  — TRUE if SDK was found and build targets created
#   AIRSIM_VERSION      — Version string (tag of vendored submodule)
#   AirSim::AirLib      — Imported static library
#   AirSim::rpc         — Imported static library (rpclib)
#   AirSim::MavLinkCom  — Imported static library
#   AirSim::All          — Interface target combining all three + Threads
# ============================================================================

include(ExternalProject)

set(AIRSIM_SUBMODULE_DIR "${CMAKE_SOURCE_DIR}/third_party/cosys-airsim")
# NOTE: Update this when bumping the submodule to a new tag.
# Dependabot PRs that update the submodule should also update this version.
set(AIRSIM_VERSION "5.5-v3.3" CACHE STRING "Cosys-AirSim SDK version (must match submodule tag)")

# ── Check if submodule is populated ──────────────────────────
# AirSim.sln is a reliable sentinel file that exists in the repo root.
if(NOT EXISTS "${AIRSIM_SUBMODULE_DIR}/AirSim.sln")
    set(COSYS_AIRSIM_FOUND FALSE)
    message(STATUS "  Cosys-AirSim : NOT FOUND — submodule not initialized")
    message(STATUS "    Hint: git submodule update --init third_party/cosys-airsim")
    return()
endif()

# ── Ensure rpclib source is available ────────────────────────
# AirSim's setup.sh normally downloads rpclib. We handle it in CMake so the
# submodule works without running setup.sh first.
set(RPCLIB_DIR "${AIRSIM_SUBMODULE_DIR}/external/rpclib/rpclib-2.3.1")
# Override for air-gapped builds or internal mirrors:
#   cmake .. -DRPCLIB_DOWNLOAD_URL=https://internal-mirror.example.com/rpclib-2.3.1.tar.gz
set(RPCLIB_DOWNLOAD_URL "https://github.com/WouterJansen/rpclib/archive/refs/tags/v2.3.1.tar.gz"
    CACHE STRING "URL for rpclib tarball (override for mirrors/air-gapped builds)")
set(RPCLIB_DOWNLOAD_DEST "${AIRSIM_SUBMODULE_DIR}/external/rpclib/v2.3.1.tar.gz")

# NOTE: rpclib is extracted into the submodule tree (external/rpclib/) because
# AirSim's own CMakeLists expects it at that exact path. This makes the submodule
# appear "dirty" (untracked files). Add external/rpclib/ to .gitignore if this
# bothers your workflow. Moving it to CMAKE_BINARY_DIR would require patching
# AirSim's build system, which defeats the purpose of vendoring.
if(NOT EXISTS "${RPCLIB_DIR}/CMakeLists.txt")
    message(STATUS "  Cosys-AirSim : Downloading rpclib v2.3.1...")

    # Create target directory
    file(MAKE_DIRECTORY "${AIRSIM_SUBMODULE_DIR}/external/rpclib")

    # Download the tarball (SHA256 pinned for supply-chain integrity)
    file(DOWNLOAD
        "${RPCLIB_DOWNLOAD_URL}"
        "${RPCLIB_DOWNLOAD_DEST}"
        STATUS _rpclib_download_status
        TIMEOUT 60
        EXPECTED_HASH SHA256=16a1f5f112e2a7f6706f660f8ab3f0937049aef5750d580311e34b0e8846d248
        TLS_VERIFY ON
    )
    list(GET _rpclib_download_status 0 _rpclib_download_code)

    if(NOT _rpclib_download_code EQUAL 0)
        list(GET _rpclib_download_status 1 _rpclib_download_msg)
        set(COSYS_AIRSIM_FOUND FALSE)
        message(STATUS "  Cosys-AirSim : NOT FOUND — rpclib download failed: ${_rpclib_download_msg}")
        message(STATUS "    Hint: manually download rpclib v2.3.1 to ${RPCLIB_DIR}/")
        return()
    endif()

    # Extract — produces rpclib-2.3.1/ directory
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xzf "${RPCLIB_DOWNLOAD_DEST}"
        WORKING_DIRECTORY "${AIRSIM_SUBMODULE_DIR}/external/rpclib"
        RESULT_VARIABLE _rpclib_extract_result
    )

    if(NOT _rpclib_extract_result EQUAL 0)
        set(COSYS_AIRSIM_FOUND FALSE)
        message(STATUS "  Cosys-AirSim : NOT FOUND — rpclib extraction failed")
        return()
    endif()

    # Verify extraction produced the expected directory
    if(NOT EXISTS "${RPCLIB_DIR}/CMakeLists.txt")
        set(COSYS_AIRSIM_FOUND FALSE)
        message(STATUS "  Cosys-AirSim : NOT FOUND — rpclib extracted but CMakeLists.txt not found")
        return()
    endif()

    message(STATUS "  Cosys-AirSim : rpclib v2.3.1 downloaded and extracted")
endif()

# ── Build via ExternalProject_Add ────────────────────────────
# AirSim's CMake entry point is in the cmake/ subdirectory. It builds:
#   - AirLib (the main client library)
#   - rpc (rpclib, for RPC transport)
#   - MavLinkCom (MAVLink communication)
# Output libraries go to <BINARY_DIR>/output/lib/

set(AIRSIM_BINARY_DIR "${CMAKE_BINARY_DIR}/airsim-build")
set(AIRSIM_OUTPUT_LIB_DIR "${AIRSIM_BINARY_DIR}/output/lib")

find_package(Threads REQUIRED)

# Handle multi-config generators (Ninja Multi-Config, VS) where
# CMAKE_BUILD_TYPE is empty — default to Release for the external build.
set(_AIRSIM_EFFECTIVE_TYPE "${CMAKE_BUILD_TYPE}")
if(NOT _AIRSIM_EFFECTIVE_TYPE)
    set(_AIRSIM_EFFECTIVE_TYPE "Release")
endif()

# Forward compiler and toolchain settings so cross-compilation and
# non-default compilers work consistently with the external build.
# If CMAKE_C_COMPILER or CMAKE_CXX_COMPILER are not set, we allow CMake
# to auto-detect them in the external build (don't pass empty strings).
set(_AIRSIM_COMPILER_ARGS)
if(CMAKE_C_COMPILER)
    list(APPEND _AIRSIM_COMPILER_ARGS -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER})
endif()
if(CMAKE_CXX_COMPILER)
    list(APPEND _AIRSIM_COMPILER_ARGS -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER})
endif()
if(CMAKE_TOOLCHAIN_FILE)
    list(APPEND _AIRSIM_COMPILER_ARGS -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE})
endif()

ExternalProject_Add(airsim_external
    SOURCE_DIR        "${AIRSIM_SUBMODULE_DIR}/cmake"
    BINARY_DIR        "${AIRSIM_BINARY_DIR}"
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=${_AIRSIM_EFFECTIVE_TYPE}
        ${_AIRSIM_COMPILER_ARGS}
        -DCMAKE_CXX_STANDARD=17
        -DCMAKE_CXX_STANDARD_REQUIRED=ON
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    BUILD_BYPRODUCTS
        "${AIRSIM_OUTPUT_LIB_DIR}/libAirLib.a"
        "${AIRSIM_OUTPUT_LIB_DIR}/librpc.a"
        "${AIRSIM_OUTPUT_LIB_DIR}/libMavLinkCom.a"
    INSTALL_COMMAND   ""
    LOG_CONFIGURE     TRUE
    LOG_BUILD         TRUE
)

# ── Create imported targets ──────────────────────────────────
# These targets depend on airsim_external so they are built before use.

# AirSim::AirLib
add_library(AirSim::AirLib STATIC IMPORTED GLOBAL)
set_target_properties(AirSim::AirLib PROPERTIES
    IMPORTED_LOCATION "${AIRSIM_OUTPUT_LIB_DIR}/libAirLib.a"
    INTERFACE_INCLUDE_DIRECTORIES
        "${AIRSIM_SUBMODULE_DIR}/AirLib/include"
)
add_dependencies(AirSim::AirLib airsim_external)

# AirSim::rpc
add_library(AirSim::rpc STATIC IMPORTED GLOBAL)
set_target_properties(AirSim::rpc PROPERTIES
    IMPORTED_LOCATION "${AIRSIM_OUTPUT_LIB_DIR}/librpc.a"
    INTERFACE_INCLUDE_DIRECTORIES
        "${RPCLIB_DIR}/include"
)
add_dependencies(AirSim::rpc airsim_external)

# AirSim::MavLinkCom
add_library(AirSim::MavLinkCom STATIC IMPORTED GLOBAL)
set_target_properties(AirSim::MavLinkCom PROPERTIES
    IMPORTED_LOCATION "${AIRSIM_OUTPUT_LIB_DIR}/libMavLinkCom.a"
    INTERFACE_INCLUDE_DIRECTORIES
        "${AIRSIM_SUBMODULE_DIR}/MavLinkCom/include"
)
add_dependencies(AirSim::MavLinkCom airsim_external)

# AirSim::All — convenience interface target combining everything
add_library(AirSim::All INTERFACE IMPORTED GLOBAL)
set_target_properties(AirSim::All PROPERTIES
    INTERFACE_LINK_LIBRARIES "AirSim::AirLib;AirSim::MavLinkCom;AirSim::rpc;Threads::Threads"
)

set(COSYS_AIRSIM_FOUND TRUE)
message(STATUS "  Cosys-AirSim : ${AIRSIM_VERSION} — Cosys-AirSim HAL backends available (ExternalProject)")
