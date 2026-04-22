// tests/test_semantic_voxels.cpp
// Unit tests for SemanticVoxel / SemanticVoxelBatch IPC types (Epic #520, Issue #608).
//
// Scope for PR 1 (IPC wire type only):
//   - Default construction zero-initialises (no uninitialised reads possible)
//   - validate() accepts well-formed voxels / batches
//   - validate() rejects NaN / Inf positions and out-of-range [0,1] fields
//   - Zenoh topic mapping /semantic_voxels → drone/perception/voxels
//   - Trivially-copyable round-trip via byte-buffer (proxy for on-wire serialisation;
//     full bus round-trip lands in PR 2 alongside the P2 publisher)
#include "ipc/ipc_types.h"
#include "ipc/zenoh_message_bus.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// Default construction
// ═══════════════════════════════════════════════════════════

TEST(SemanticVoxel, DefaultConstructZeroInit) {
    SemanticVoxel v{};
    EXPECT_FLOAT_EQ(v.position_x, 0.0f);
    EXPECT_FLOAT_EQ(v.position_y, 0.0f);
    EXPECT_FLOAT_EQ(v.position_z, 0.0f);
    EXPECT_FLOAT_EQ(v.occupancy, 0.0f);
    EXPECT_FLOAT_EQ(v.confidence, 0.0f);
    EXPECT_EQ(v.semantic_label, ObjectClass::UNKNOWN);
    EXPECT_EQ(v.timestamp_ns, 0u);
}

TEST(SemanticVoxelBatch, DefaultConstructZeroInit) {
    auto b = std::make_unique<SemanticVoxelBatch>();
    EXPECT_EQ(b->timestamp_ns, 0u);
    EXPECT_EQ(b->frame_sequence, 0u);
    EXPECT_EQ(b->num_voxels, 0u);
    // Spot-check first, middle, and last voxel slots are zero-initialised.
    EXPECT_FLOAT_EQ(b->voxels[0].position_x, 0.0f);
    EXPECT_FLOAT_EQ(b->voxels[MAX_VOXELS_PER_BATCH / 2].confidence, 0.0f);
    EXPECT_EQ(b->voxels[MAX_VOXELS_PER_BATCH - 1].semantic_label, ObjectClass::UNKNOWN);
}

// ═══════════════════════════════════════════════════════════
// SemanticVoxel::validate()
// ═══════════════════════════════════════════════════════════

TEST(SemanticVoxel, ValidateAcceptsWellFormed) {
    SemanticVoxel v{};
    v.position_x     = 5.0f;
    v.position_y     = -3.0f;
    v.position_z     = 2.0f;
    v.occupancy      = 0.9f;
    v.confidence     = 0.7f;
    v.semantic_label = ObjectClass::GEOMETRIC_OBSTACLE;
    v.timestamp_ns   = 1'700'000'000'000'000'000ULL;
    EXPECT_TRUE(v.validate());
}

TEST(SemanticVoxel, ValidateRejectsNaNPosition) {
    SemanticVoxel v{};
    v.position_x = std::numeric_limits<float>::quiet_NaN();
    v.occupancy  = 0.5f;
    v.confidence = 0.5f;
    EXPECT_FALSE(v.validate());
}

TEST(SemanticVoxel, ValidateRejectsInfPosition) {
    SemanticVoxel v{};
    v.position_z = std::numeric_limits<float>::infinity();
    v.occupancy  = 0.5f;
    v.confidence = 0.5f;
    EXPECT_FALSE(v.validate());
}

TEST(SemanticVoxel, ValidateRejectsOccupancyAboveOne) {
    SemanticVoxel v{};
    v.occupancy  = 1.5f;
    v.confidence = 0.5f;
    EXPECT_FALSE(v.validate());
}

TEST(SemanticVoxel, ValidateRejectsNegativeOccupancy) {
    SemanticVoxel v{};
    v.occupancy  = -0.1f;
    v.confidence = 0.5f;
    EXPECT_FALSE(v.validate());
}

TEST(SemanticVoxel, ValidateRejectsConfidenceAboveOne) {
    SemanticVoxel v{};
    v.occupancy  = 0.5f;
    v.confidence = 1.01f;
    EXPECT_FALSE(v.validate());
}

// ═══════════════════════════════════════════════════════════
// SemanticVoxelBatch::validate()
// ═══════════════════════════════════════════════════════════

TEST(SemanticVoxelBatch, ValidateAcceptsEmptyBatch) {
    auto b         = std::make_unique<SemanticVoxelBatch>();
    b->num_voxels  = 0;
    EXPECT_TRUE(b->validate());
}

TEST(SemanticVoxelBatch, ValidateAcceptsPopulatedBatch) {
    auto b            = std::make_unique<SemanticVoxelBatch>();
    b->timestamp_ns   = 12345;
    b->frame_sequence = 42;
    b->num_voxels     = 3;
    for (uint32_t i = 0; i < b->num_voxels; ++i) {
        b->voxels[i].position_x     = static_cast<float>(i);
        b->voxels[i].occupancy      = 0.5f;
        b->voxels[i].confidence     = 0.8f;
        b->voxels[i].semantic_label = ObjectClass::GEOMETRIC_OBSTACLE;
    }
    EXPECT_TRUE(b->validate());
}

TEST(SemanticVoxelBatch, ValidateRejectsCountOverMax) {
    auto b        = std::make_unique<SemanticVoxelBatch>();
    b->num_voxels = MAX_VOXELS_PER_BATCH + 1;
    EXPECT_FALSE(b->validate());
}

TEST(SemanticVoxelBatch, ValidateRejectsBadVoxelInBatch) {
    auto b            = std::make_unique<SemanticVoxelBatch>();
    b->num_voxels     = 2;
    b->voxels[0].occupancy  = 0.5f;
    b->voxels[0].confidence = 0.5f;
    // Second voxel has NaN position — batch must reject.
    b->voxels[1].position_x = std::numeric_limits<float>::quiet_NaN();
    b->voxels[1].occupancy  = 0.5f;
    b->voxels[1].confidence = 0.5f;
    EXPECT_FALSE(b->validate());
}

TEST(SemanticVoxelBatch, ValidateIgnoresVoxelsPastCount) {
    // num_voxels=1; voxels[5] is NaN but shouldn't be inspected.
    auto b                = std::make_unique<SemanticVoxelBatch>();
    b->num_voxels         = 1;
    b->voxels[0].occupancy  = 0.5f;
    b->voxels[0].confidence = 0.5f;
    b->voxels[5].position_x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_TRUE(b->validate());
}

// ═══════════════════════════════════════════════════════════
// Zenoh topic mapping
// ═══════════════════════════════════════════════════════════

TEST(SemanticVoxelsTopicMapping, LegacyToKeyExpr) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr(topics::SEMANTIC_VOXELS), "drone/perception/voxels");
}

TEST(SemanticVoxelsTopicMapping, ConstantMatchesLegacyName) {
    EXPECT_STREQ(topics::SEMANTIC_VOXELS, "/semantic_voxels");
}

// ═══════════════════════════════════════════════════════════
// Trivially-copyable byte round-trip (proxy for wire serialisation;
// full bus round-trip lands with the P2 publisher in PR 2).
// ═══════════════════════════════════════════════════════════

TEST(SemanticVoxelBatch, ByteRoundTripPreservesContent) {
    auto src            = std::make_unique<SemanticVoxelBatch>();
    src->timestamp_ns   = 0xDEADBEEFCAFEULL;
    src->frame_sequence = 1234567;
    src->num_voxels     = 5;
    for (uint32_t i = 0; i < src->num_voxels; ++i) {
        src->voxels[i].position_x     = 1.0f + static_cast<float>(i);
        src->voxels[i].position_y     = 2.0f + static_cast<float>(i);
        src->voxels[i].position_z     = 3.0f + static_cast<float>(i);
        src->voxels[i].occupancy      = 0.5f;
        src->voxels[i].confidence     = 0.9f;
        src->voxels[i].semantic_label = ObjectClass::GEOMETRIC_OBSTACLE;
        src->voxels[i].timestamp_ns   = 1000ULL + i;
    }

    // Serialise: copy into a byte buffer.
    std::vector<uint8_t> wire(sizeof(SemanticVoxelBatch));
    std::memcpy(wire.data(), src.get(), wire.size());

    // Deserialise into a fresh instance.
    auto dst = std::make_unique<SemanticVoxelBatch>();
    std::memcpy(dst.get(), wire.data(), wire.size());

    EXPECT_EQ(dst->timestamp_ns, src->timestamp_ns);
    EXPECT_EQ(dst->frame_sequence, src->frame_sequence);
    EXPECT_EQ(dst->num_voxels, src->num_voxels);
    for (uint32_t i = 0; i < dst->num_voxels; ++i) {
        EXPECT_FLOAT_EQ(dst->voxels[i].position_x, src->voxels[i].position_x);
        EXPECT_FLOAT_EQ(dst->voxels[i].position_y, src->voxels[i].position_y);
        EXPECT_FLOAT_EQ(dst->voxels[i].position_z, src->voxels[i].position_z);
        EXPECT_FLOAT_EQ(dst->voxels[i].occupancy, src->voxels[i].occupancy);
        EXPECT_FLOAT_EQ(dst->voxels[i].confidence, src->voxels[i].confidence);
        EXPECT_EQ(dst->voxels[i].semantic_label, src->voxels[i].semantic_label);
        EXPECT_EQ(dst->voxels[i].timestamp_ns, src->voxels[i].timestamp_ns);
    }
    EXPECT_TRUE(dst->validate());
}
