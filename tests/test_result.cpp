// tests/test_result.cpp
// Unit tests for drone::util::Result<T, E> and Error types.
#include "util/result.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

using drone::util::Error;
using drone::util::ErrorCode;
using drone::util::Result;
using drone::util::VoidResult;

// ═══════════════════════════════════════════════════════════
// Error type tests
// ═══════════════════════════════════════════════════════════

TEST(ErrorTest, DefaultConstruction) {
    Error e;
    EXPECT_EQ(e.code(), ErrorCode::Unknown);
    EXPECT_TRUE(e.message().empty());
}

TEST(ErrorTest, CodeAndMessage) {
    Error e(ErrorCode::FileNotFound, "config.json not found");
    EXPECT_EQ(e.code(), ErrorCode::FileNotFound);
    EXPECT_EQ(e.message(), "config.json not found");
}

TEST(ErrorTest, MessageOnly) {
    Error e("something went wrong");
    EXPECT_EQ(e.code(), ErrorCode::Unknown);
    EXPECT_EQ(e.message(), "something went wrong");
}

TEST(ErrorTest, EqualityOperators) {
    Error a(ErrorCode::ParseError, "bad json");
    Error b(ErrorCode::ParseError, "bad json");
    Error c(ErrorCode::ParseError, "different");
    Error d(ErrorCode::MissingKey, "bad json");

    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
    EXPECT_NE(a, d);
}

// ═══════════════════════════════════════════════════════════
// Result<T> — value path
// ═══════════════════════════════════════════════════════════

TEST(ResultTest, OkInt) {
    auto r = Result<int>::ok(42);
    EXPECT_TRUE(r.is_ok());
    EXPECT_FALSE(r.is_err());
    EXPECT_EQ(r.value(), 42);
}

TEST(ResultTest, OkString) {
    auto r = Result<std::string>::ok("hello");
    EXPECT_TRUE(r.is_ok());
    EXPECT_EQ(r.value(), "hello");
}

TEST(ResultTest, OkVector) {
    auto r = Result<std::vector<int>>::ok({1, 2, 3});
    EXPECT_TRUE(r.is_ok());
    EXPECT_EQ(r.value().size(), 3u);
    EXPECT_EQ(r.value()[2], 3);
}

// ═══════════════════════════════════════════════════════════
// Result<T> — error path
// ═══════════════════════════════════════════════════════════

TEST(ResultTest, ErrValue) {
    auto r = Result<int>::err(Error(ErrorCode::InvalidValue, "negative port"));
    EXPECT_TRUE(r.is_err());
    EXPECT_FALSE(r.is_ok());
    EXPECT_EQ(r.error().code(), ErrorCode::InvalidValue);
    EXPECT_EQ(r.error().message(), "negative port");
}

// ═══════════════════════════════════════════════════════════
// value_or
// ═══════════════════════════════════════════════════════════

TEST(ResultTest, ValueOrOnOk) {
    auto r = Result<int>::ok(10);
    EXPECT_EQ(r.value_or(99), 10);
}

TEST(ResultTest, ValueOrOnErr) {
    auto r = Result<int>::err(Error("fail"));
    EXPECT_EQ(r.value_or(99), 99);
}

TEST(ResultTest, ValueOrMoveSemantics) {
    auto r = Result<std::string>::ok("moved");
    auto v = std::move(r).value_or("fallback");
    EXPECT_EQ(v, "moved");
}

TEST(ResultTest, ValueOrMoveOnErr) {
    auto r = Result<std::string>::err(Error("nope"));
    auto v = std::move(r).value_or("fallback");
    EXPECT_EQ(v, "fallback");
}

// ═══════════════════════════════════════════════════════════
// map()
// ═══════════════════════════════════════════════════════════

TEST(ResultTest, MapOnOk) {
    auto r = Result<int>::ok(5);
    auto doubled = r.map([](int x) { return x * 2; });
    EXPECT_TRUE(doubled.is_ok());
    EXPECT_EQ(doubled.value(), 10);
}

TEST(ResultTest, MapOnErr) {
    auto r = Result<int>::err(Error("bad"));
    auto doubled = r.map([](int x) { return x * 2; });
    EXPECT_TRUE(doubled.is_err());
    EXPECT_EQ(doubled.error().message(), "bad");
}

TEST(ResultTest, MapChangesType) {
    auto r   = Result<int>::ok(42);
    auto str = r.map([](int x) { return std::to_string(x); });
    EXPECT_TRUE(str.is_ok());
    EXPECT_EQ(str.value(), "42");
}

TEST(ResultTest, MapMoveOverload) {
    auto r   = Result<std::string>::ok("hello");
    auto len = std::move(r).map([](std::string&& s) { return s.size(); });
    EXPECT_TRUE(len.is_ok());
    EXPECT_EQ(len.value(), 5u);
}

// ═══════════════════════════════════════════════════════════
// and_then() — flat map
// ═══════════════════════════════════════════════════════════

TEST(ResultTest, AndThenOnOk) {
    auto parse = [](int x) -> Result<double> {
        if (x >= 0) return Result<double>::ok(static_cast<double>(x) / 10.0);
        return Result<double>::err(Error(ErrorCode::InvalidValue, "negative"));
    };

    auto r = Result<int>::ok(50);
    auto d = r.and_then(parse);
    EXPECT_TRUE(d.is_ok());
    EXPECT_DOUBLE_EQ(d.value(), 5.0);
}

TEST(ResultTest, AndThenPropagatesError) {
    auto parse = [](int x) -> Result<double> {
        if (x >= 0) return Result<double>::ok(static_cast<double>(x));
        return Result<double>::err(Error("negative"));
    };

    auto r = Result<int>::err(Error("upstream fail"));
    auto d = r.and_then(parse);
    EXPECT_TRUE(d.is_err());
    EXPECT_EQ(d.error().message(), "upstream fail");
}

TEST(ResultTest, AndThenReturnsNewError) {
    auto parse = [](int /*x*/) -> Result<double> {
        return Result<double>::err(Error(ErrorCode::OutOfRange, "too big"));
    };

    auto r = Result<int>::ok(99999);
    auto d = r.and_then(parse);
    EXPECT_TRUE(d.is_err());
    EXPECT_EQ(d.error().code(), ErrorCode::OutOfRange);
}

TEST(ResultTest, AndThenChaining) {
    auto step1 = [](int x) -> Result<int> {
        return Result<int>::ok(x + 1);
    };
    auto step2 = [](int x) -> Result<std::string> {
        return Result<std::string>::ok("val=" + std::to_string(x));
    };

    auto result = Result<int>::ok(10).and_then(step1).and_then(step2);
    EXPECT_TRUE(result.is_ok());
    EXPECT_EQ(result.value(), "val=11");
}

// ═══════════════════════════════════════════════════════════
// map_error()
// ═══════════════════════════════════════════════════════════

TEST(ResultTest, MapErrorOnErr) {
    auto r = Result<int>::err(Error(ErrorCode::ParseError, "bad json"));
    auto mapped =
        r.map_error([](const Error& e) { return Error(e.code(), "wrapped: " + e.message()); });
    EXPECT_TRUE(mapped.is_err());
    EXPECT_EQ(mapped.error().message(), "wrapped: bad json");
}

TEST(ResultTest, MapErrorOnOk) {
    auto r      = Result<int>::ok(42);
    auto mapped = r.map_error([](const Error& e) { return Error(e.code(), "wrapped"); });
    EXPECT_TRUE(mapped.is_ok());
    EXPECT_EQ(mapped.value(), 42);
}

// ═══════════════════════════════════════════════════════════
// Move semantics
// ═══════════════════════════════════════════════════════════

TEST(ResultTest, MoveValue) {
    auto r = Result<std::string>::ok("moveable");
    auto s = std::move(r).value();
    EXPECT_EQ(s, "moveable");
}

TEST(ResultTest, MoveError) {
    auto r = Result<int>::err(Error("moveable error"));
    auto e = std::move(r).error();
    EXPECT_EQ(e.message(), "moveable error");
}

TEST(ResultTest, CopyResult) {
    auto r1 = Result<int>::ok(7);
    auto r2 = r1;  // NOLINT — intentional copy
    EXPECT_TRUE(r2.is_ok());
    EXPECT_EQ(r2.value(), 7);
    EXPECT_EQ(r1.value(), 7);  // original unmodified
}

// ═══════════════════════════════════════════════════════════
// VoidResult (Result<void, Error>)
// ═══════════════════════════════════════════════════════════

TEST(VoidResultTest, Ok) {
    auto r = VoidResult::ok();
    EXPECT_TRUE(r.is_ok());
    EXPECT_FALSE(r.is_err());
}

TEST(VoidResultTest, Err) {
    auto r = VoidResult::err(Error(ErrorCode::NotConnected, "link down"));
    EXPECT_TRUE(r.is_err());
    EXPECT_EQ(r.error().code(), ErrorCode::NotConnected);
    EXPECT_EQ(r.error().message(), "link down");
}

TEST(VoidResultTest, AndThenOnOk) {
    int called = 0;
    auto r     = VoidResult::ok();
    auto r2    = r.and_then([&]() -> VoidResult {
        called++;
        return VoidResult::ok();
    });
    EXPECT_EQ(called, 1);
    EXPECT_TRUE(r2.is_ok());
}

TEST(VoidResultTest, AndThenOnErr) {
    int called = 0;
    auto r     = VoidResult::err(Error("fail"));
    auto r2    = r.and_then([&]() -> VoidResult {
        called++;
        return VoidResult::ok();
    });
    EXPECT_EQ(called, 0);
    EXPECT_TRUE(r2.is_err());
    EXPECT_EQ(r2.error().message(), "fail");
}

// ═══════════════════════════════════════════════════════════
// Custom error type
// ═══════════════════════════════════════════════════════════

struct AppError {
    int         code;
    std::string detail;
};

TEST(ResultTest, CustomErrorType) {
    auto r = Result<int, AppError>::err(AppError{404, "not found"});
    EXPECT_TRUE(r.is_err());
    EXPECT_EQ(r.error().code, 404);
    EXPECT_EQ(r.error().detail, "not found");
}

TEST(ResultTest, CustomErrorMapToDefault) {
    auto r = Result<int, AppError>::err(AppError{500, "server error"});
    EXPECT_EQ(r.value_or(0), 0);
}

// ═══════════════════════════════════════════════════════════
// map() with void-returning callable
// ═══════════════════════════════════════════════════════════

TEST(ResultTest, MapVoidReturnOk) {
    auto r = Result<int>::ok(42);
    int captured = 0;
    auto v = r.map([&](int x) { captured = x; });
    static_assert(std::is_same_v<decltype(v), Result<void>>);
    EXPECT_TRUE(v.is_ok());
    EXPECT_EQ(captured, 42);
}

TEST(ResultTest, MapVoidReturnErr) {
    auto r = Result<int>::err(Error{ErrorCode::Unknown, "fail"});
    bool called = false;
    auto v = r.map([&](int) { called = true; });
    EXPECT_TRUE(v.is_err());
    EXPECT_FALSE(called);
}

TEST(ResultTest, MapVoidReturnRvalue) {
    auto v = Result<std::string>::ok("hello").map([](std::string&&) {});
    EXPECT_TRUE(v.is_ok());
}

// ═══════════════════════════════════════════════════════════
// ErrorCode coverage
// ═══════════════════════════════════════════════════════════

TEST(ErrorCodeTest, AllCodes) {
    // Verify all error codes are distinct
    std::vector<ErrorCode> codes = {
        ErrorCode::Unknown,    ErrorCode::FileNotFound,  ErrorCode::ParseError,
        ErrorCode::InvalidValue, ErrorCode::MissingKey,  ErrorCode::TypeMismatch,
        ErrorCode::OutOfRange, ErrorCode::Timeout,       ErrorCode::NotConnected,
        ErrorCode::AlreadyExists,
    };
    for (size_t i = 0; i < codes.size(); ++i) {
        for (size_t j = i + 1; j < codes.size(); ++j) {
            EXPECT_NE(codes[i], codes[j]) << "codes[" << i << "] == codes[" << j << "]";
        }
    }
}
