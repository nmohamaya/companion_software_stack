// tests/test_correlation.cpp
// Unit tests for cross-process correlation ID support.
//
// Covers:
//   - CorrelationContext generate/set/get/clear
//   - ScopedCorrelation RAII guard
//   - Thread isolation of correlation IDs
//   - SHM message types have correlation_id field
//   - WireHeader v2 layout (32 bytes, correlation_id)
//   - WireHeader backward-compatible validation (v1 → v2)
//   - JSON log sink includes correlation_id when non-zero

#include "ipc/ipc_types.h"
#include "ipc/wire_format.h"
#include "util/correlation.h"
#include "util/json_log_sink.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::util;
using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// CorrelationContext — basic operations
// ═══════════════════════════════════════════════════════════

TEST(CorrelationContext, InitiallyZero) {
    CorrelationContext::clear();
    EXPECT_EQ(CorrelationContext::get(), 0ULL);
}

TEST(CorrelationContext, SetAndGet) {
    CorrelationContext::clear();
    CorrelationContext::set(0xDEADBEEF);
    EXPECT_EQ(CorrelationContext::get(), 0xDEADBEEF);
    CorrelationContext::clear();
}

TEST(CorrelationContext, Clear) {
    CorrelationContext::set(42);
    CorrelationContext::clear();
    EXPECT_EQ(CorrelationContext::get(), 0ULL);
}

TEST(CorrelationContext, GenerateNonZero) {
    CorrelationContext::clear();
    auto id = CorrelationContext::generate();
    EXPECT_NE(id, 0ULL);
    EXPECT_EQ(CorrelationContext::get(), id);
    CorrelationContext::clear();
}

TEST(CorrelationContext, GenerateMonotonic) {
    std::vector<uint64_t> ids;
    for (int i = 0; i < 100; ++i) {
        ids.push_back(CorrelationContext::generate());
    }
    // Lower 32 bits should be monotonically increasing
    for (std::size_t i = 1; i < ids.size(); ++i) {
        uint32_t lo_prev = static_cast<uint32_t>(ids[i - 1] & 0xFFFFFFFF);
        uint32_t lo_cur  = static_cast<uint32_t>(ids[i] & 0xFFFFFFFF);
        EXPECT_GT(lo_cur, lo_prev);
    }
    CorrelationContext::clear();
}

TEST(CorrelationContext, GenerateContainsPID) {
    auto     id    = CorrelationContext::generate();
    uint32_t hi    = static_cast<uint32_t>(id >> 32);
    uint32_t mypid = static_cast<uint32_t>(getpid());
    EXPECT_EQ(hi, mypid);
    CorrelationContext::clear();
}

TEST(CorrelationContext, GenerateUnique) {
    const int             n = 1000;
    std::vector<uint64_t> ids;
    ids.reserve(n);
    for (int i = 0; i < n; ++i) {
        ids.push_back(CorrelationContext::generate());
    }
    // All IDs should be unique
    std::sort(ids.begin(), ids.end());
    auto last = std::unique(ids.begin(), ids.end());
    EXPECT_EQ(last, ids.end());
    CorrelationContext::clear();
}

// ═══════════════════════════════════════════════════════════
// ScopedCorrelation — RAII guard
// ═══════════════════════════════════════════════════════════

TEST(ScopedCorrelation, SetsAndRestores) {
    CorrelationContext::set(100);
    {
        ScopedCorrelation guard(200);
        EXPECT_EQ(CorrelationContext::get(), 200ULL);
    }
    EXPECT_EQ(CorrelationContext::get(), 100ULL);
    CorrelationContext::clear();
}

TEST(ScopedCorrelation, NestedGuards) {
    CorrelationContext::clear();
    {
        ScopedCorrelation outer(10);
        EXPECT_EQ(CorrelationContext::get(), 10ULL);
        {
            ScopedCorrelation inner(20);
            EXPECT_EQ(CorrelationContext::get(), 20ULL);
        }
        EXPECT_EQ(CorrelationContext::get(), 10ULL);
    }
    EXPECT_EQ(CorrelationContext::get(), 0ULL);
}

TEST(ScopedCorrelation, RestoresZero) {
    CorrelationContext::clear();
    {
        ScopedCorrelation guard(42);
        EXPECT_EQ(CorrelationContext::get(), 42ULL);
    }
    EXPECT_EQ(CorrelationContext::get(), 0ULL);
}

// ═══════════════════════════════════════════════════════════
// Thread isolation
// ═══════════════════════════════════════════════════════════

TEST(CorrelationContext, ThreadIsolation) {
    CorrelationContext::set(111);

    std::atomic<uint64_t> thread_id{0};
    std::thread           t([&] {
        // New thread should start with 0 (thread-local)
        thread_id.store(CorrelationContext::get(), std::memory_order_release);
        CorrelationContext::set(222);
    });
    t.join();

    EXPECT_EQ(thread_id.load(), 0ULL);
    EXPECT_EQ(CorrelationContext::get(), 111ULL);  // main thread unaffected
    CorrelationContext::clear();
}

TEST(CorrelationContext, MultiThreadGenerate) {
    const int                n = 4;
    std::vector<std::thread> threads;
    std::vector<uint64_t>    ids(n * 100);
    std::atomic<int>         idx{0};

    for (int t = 0; t < n; ++t) {
        threads.emplace_back([&] {
            for (int i = 0; i < 100; ++i) {
                auto id   = CorrelationContext::generate();
                int  slot = idx.fetch_add(1, std::memory_order_relaxed);
                ids[slot] = id;
            }
        });
    }
    for (auto& t : threads) t.join();

    // All IDs unique across threads (same PID but different counter values)
    std::sort(ids.begin(), ids.end());
    auto last = std::unique(ids.begin(), ids.end());
    EXPECT_EQ(last, ids.end());
    CorrelationContext::clear();
}

// ═══════════════════════════════════════════════════════════
// IPC message types — correlation_id field
// ═══════════════════════════════════════════════════════════

TEST(IpcCorrelation, GCSCommandHasCorrelationId) {
    GCSCommand cmd{};
    cmd.correlation_id = 0xABCD1234;
    EXPECT_EQ(cmd.correlation_id, 0xABCD1234ULL);
    static_assert(std::is_trivially_copyable_v<GCSCommand>);
}

TEST(IpcCorrelation, FCCommandHasCorrelationId) {
    FCCommand cmd{};
    cmd.correlation_id = 0xFACE;
    EXPECT_EQ(cmd.correlation_id, 0xFACEULL);
    static_assert(std::is_trivially_copyable_v<FCCommand>);
}

TEST(IpcCorrelation, TrajectoryCmdHasCorrelationId) {
    TrajectoryCmd cmd{};
    cmd.correlation_id = 0x12345678;
    EXPECT_EQ(cmd.correlation_id, 0x12345678ULL);
    static_assert(std::is_trivially_copyable_v<TrajectoryCmd>);
}

TEST(IpcCorrelation, PayloadCommandHasCorrelationId) {
    PayloadCommand cmd{};
    cmd.correlation_id = 0xBEEF;
    EXPECT_EQ(cmd.correlation_id, 0xBEEFULL);
    static_assert(std::is_trivially_copyable_v<PayloadCommand>);
}

TEST(IpcCorrelation, MissionStatusHasCorrelationId) {
    MissionStatus status{};
    status.correlation_id = 0xCAFE;
    EXPECT_EQ(status.correlation_id, 0xCAFEULL);
    static_assert(std::is_trivially_copyable_v<MissionStatus>);
}

TEST(IpcCorrelation, DefaultZero) {
    GCSCommand     gcs{};
    FCCommand      fc{};
    TrajectoryCmd  traj{};
    PayloadCommand pay{};
    MissionStatus  ms{};
    EXPECT_EQ(gcs.correlation_id, 0ULL);
    EXPECT_EQ(fc.correlation_id, 0ULL);
    EXPECT_EQ(traj.correlation_id, 0ULL);
    EXPECT_EQ(pay.correlation_id, 0ULL);
    EXPECT_EQ(ms.correlation_id, 0ULL);
}

// ═══════════════════════════════════════════════════════════
// WireHeader v2 — layout and size
// ═══════════════════════════════════════════════════════════

TEST(WireHeaderV2, SizeIs32Bytes) {
    EXPECT_EQ(sizeof(WireHeader), 32U);
}

TEST(WireHeaderV2, VersionIs3) {
    WireHeader hdr;
    EXPECT_EQ(hdr.version, 3);
    EXPECT_EQ(kWireVersion, 3);
}

TEST(WireHeaderV2, CorrelationIdDefaultZero) {
    WireHeader hdr;
    EXPECT_EQ(hdr.correlation_id, 0ULL);
}

TEST(WireHeaderV2, CorrelationIdRoundTrip) {
    WireHeader hdr;
    hdr.correlation_id = 0xDEADBEEFCAFEBABE;
    hdr.msg_type       = WireMessageType::FC_COMMAND;
    hdr.payload_size   = 64;
    hdr.sequence       = 42;

    uint8_t buf[sizeof(WireHeader)];
    std::memcpy(buf, &hdr, sizeof(WireHeader));

    WireHeader decoded = wire_read_header(buf);
    EXPECT_EQ(decoded.correlation_id, 0xDEADBEEFCAFEBABEULL);
    EXPECT_EQ(decoded.msg_type, WireMessageType::FC_COMMAND);
    EXPECT_EQ(decoded.sequence, 42U);
}

TEST(WireHeaderV2, SerializeDeserializeWithCorrelation) {
    FCCommand cmd{};
    cmd.timestamp_ns   = 123456789;
    cmd.correlation_id = 0xABCDEF01;
    cmd.command        = FCCommandType::RTL;
    cmd.sequence_id    = 7;
    cmd.valid          = true;

    auto buf = wire_serialize(cmd, WireMessageType::FC_COMMAND, 1);
    ASSERT_TRUE(wire_validate(buf.data(), buf.size()));

    auto hdr = wire_read_header(buf.data());
    EXPECT_EQ(hdr.correlation_id, 0ULL);  // WireHeader carries its own correlation_id
    // The SHM struct correlation_id is in the payload, not the header
    // To test header correlation, set it via serialize:

    // Manually set header correlation_id
    WireHeader hdr2;
    hdr2.correlation_id = 0xFEED;
    hdr2.msg_type       = WireMessageType::FC_COMMAND;
    hdr2.payload_size   = sizeof(FCCommand);
    hdr2.sequence       = 1;

    std::vector<uint8_t> buf2(sizeof(WireHeader) + sizeof(FCCommand));
    std::memcpy(buf2.data(), &hdr2, sizeof(WireHeader));
    std::memcpy(buf2.data() + sizeof(WireHeader), &cmd, sizeof(FCCommand));

    ASSERT_TRUE(wire_validate(buf2.data(), buf2.size()));
    auto decoded_hdr = wire_read_header(buf2.data());
    EXPECT_EQ(decoded_hdr.correlation_id, 0xFEEDULL);

    FCCommand decoded_cmd{};
    ASSERT_TRUE(wire_deserialize(buf2.data(), buf2.size(), decoded_cmd));
    EXPECT_EQ(decoded_cmd.correlation_id, 0xABCDEF01ULL);
}

// ═══════════════════════════════════════════════════════════
// WireHeader backward compatibility — v1 messages
// ═══════════════════════════════════════════════════════════

TEST(WireHeaderBackcompat, V1HeaderValidates) {
    // Construct a v1-style header (24 bytes) + dummy payload
    struct __attribute__((packed)) V1Header {
        uint32_t magic        = kWireMagic;
        uint8_t  version      = 1;
        uint8_t  flags        = 0;
        uint16_t msg_type     = 33;  // FC_COMMAND
        uint32_t payload_size = 4;
        uint64_t timestamp_ns = 0;
        uint32_t sequence     = 0;
    };
    static_assert(sizeof(V1Header) == 24);

    V1Header v1;
    v1.payload_size = 4;

    uint8_t buf[28] = {};
    std::memcpy(buf, &v1, 24);
    // 4 bytes dummy payload
    buf[24] = 0x01;

    EXPECT_TRUE(wire_validate(buf, 28));
}

TEST(WireHeaderBackcompat, V1HeaderCorrelationIdIsZero) {
    struct __attribute__((packed)) V1Header {
        uint32_t magic        = kWireMagic;
        uint8_t  version      = 1;
        uint8_t  flags        = 0;
        uint16_t msg_type     = 33;
        uint32_t payload_size = 0;
        uint64_t timestamp_ns = 99;
        uint32_t sequence     = 5;
    };
    V1Header v1;

    uint8_t buf[24] = {};
    std::memcpy(buf, &v1, 24);

    auto hdr = wire_read_header(buf);
    EXPECT_EQ(hdr.correlation_id, 0ULL);
    EXPECT_EQ(hdr.sequence, 5U);
    EXPECT_EQ(hdr.timestamp_ns, 99ULL);
}

TEST(WireHeaderBackcompat, V0Rejected) {
    uint8_t  buf[32] = {};
    uint32_t magic   = kWireMagic;
    std::memcpy(buf, &magic, 4);
    buf[4] = 0;  // version 0
    EXPECT_FALSE(wire_validate(buf, 32));
}

TEST(WireHeaderBackcompat, FutureVersionRejected) {
    uint8_t  buf[32] = {};
    uint32_t magic   = kWireMagic;
    std::memcpy(buf, &magic, 4);
    buf[4] = 4;  // version 4 (future, > kWireVersion)
    EXPECT_FALSE(wire_validate(buf, 32));
}

TEST(WireHeaderBackcompat, TruncatedV1Rejected) {
    uint8_t  buf[20] = {};
    uint32_t magic   = kWireMagic;
    std::memcpy(buf, &magic, 4);
    buf[4] = 1;
    EXPECT_FALSE(wire_validate(buf, 20));  // < 24 bytes
}

TEST(WireHeaderBackcompat, TruncatedV2Rejected) {
    uint8_t  buf[28] = {};
    uint32_t magic   = kWireMagic;
    std::memcpy(buf, &magic, 4);
    buf[4] = 2;
    EXPECT_FALSE(wire_validate(buf, 28));  // < 32 bytes
}

// ═══════════════════════════════════════════════════════════
// JSON log sink — correlation_id field
// ═══════════════════════════════════════════════════════════

// RAII wrapper for tmpfile() to avoid FD leaks in tests.
struct FileCloser {
    void operator()(FILE* f) const {
        if (f) std::fclose(f);
    }
};
using FilePtr = std::unique_ptr<FILE, FileCloser>;

TEST(JsonCorrelation, OmittedWhenZero) {
    CorrelationContext::clear();
    FilePtr fp(tmpfile());
    auto    sink   = std::make_shared<JsonLogSink_st>(fp.get());
    auto    logger = std::make_shared<spdlog::logger>("test_corr_zero", sink);

    logger->info("no correlation");

    auto json = sink->last_json();
    EXPECT_EQ(json.find("correlation_id"), std::string::npos);
    CorrelationContext::clear();
}

TEST(JsonCorrelation, PresentWhenNonZero) {
    CorrelationContext::set(0x00010000ABCD0001);
    FilePtr fp(tmpfile());
    auto    sink   = std::make_shared<JsonLogSink_st>(fp.get());
    auto    logger = std::make_shared<spdlog::logger>("test_corr_set", sink);

    logger->info("with correlation");

    auto json = sink->last_json();
    EXPECT_NE(json.find("correlation_id"), std::string::npos);
    EXPECT_NE(json.find("0x00010000abcd0001"), std::string::npos);
    CorrelationContext::clear();
}

TEST(JsonCorrelation, ScopedGuardAffectsLogOutput) {
    CorrelationContext::clear();
    FilePtr fp(tmpfile());
    auto    sink   = std::make_shared<JsonLogSink_st>(fp.get());
    auto    logger = std::make_shared<spdlog::logger>("test_corr_scoped", sink);

    {
        ScopedCorrelation guard(0xFACEFEED);
        logger->info("inside scope");
        auto json = sink->last_json();
        EXPECT_NE(json.find("0x00000000facefeed"), std::string::npos);
    }

    logger->info("outside scope");
    auto json = sink->last_json();
    EXPECT_EQ(json.find("correlation_id"), std::string::npos);
}

TEST(JsonCorrelation, CorrelationIdIsHexString) {
    CorrelationContext::set(255);
    FilePtr fp(tmpfile());
    auto    sink   = std::make_shared<JsonLogSink_st>(fp.get());
    auto    logger = std::make_shared<spdlog::logger>("test_corr_hex", sink);

    logger->info("hex test");

    auto json = sink->last_json();
    // Should be padded hex: "0x00000000000000ff"
    EXPECT_NE(json.find("\"correlation_id\":\"0x00000000000000ff\""), std::string::npos);
    CorrelationContext::clear();
}
