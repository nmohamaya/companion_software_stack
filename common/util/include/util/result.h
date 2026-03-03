// common/util/include/util/result.h
// Lightweight Result<T, E> type for monadic error handling.
//
// Usage:
//   Result<int, Error> parse_port(const std::string& s);
//   auto r = parse_port("8080");
//   if (r.is_ok()) use(r.value());
//   else           log(r.error().message());
//
//   // Chaining:
//   auto v = parse_port(s)
//              .map([](int p) { return p + 1; })
//              .value_or(9090);
//
// Design notes:
//   - C++17, header-only, zero heap allocation
//   - Uses std::variant internally
//   - All Result-returning functions should be [[nodiscard]]
#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace drone::util {

// ── Error type ──────────────────────────────────────────────
// Simple value-type carrying an error code and a human-readable message.

enum class ErrorCode : uint8_t {
    Unknown       = 0,
    FileNotFound  = 1,
    ParseError    = 2,
    InvalidValue  = 3,
    MissingKey    = 4,
    TypeMismatch  = 5,
    OutOfRange    = 6,
    Timeout       = 7,
    NotConnected  = 8,
    AlreadyExists = 9,
};

class Error {
public:
    Error() = default;

    Error(ErrorCode code, std::string message) : code_(code), message_(std::move(message)) {}

    explicit Error(std::string message) : code_(ErrorCode::Unknown), message_(std::move(message)) {}

    [[nodiscard]] ErrorCode          code() const { return code_; }
    [[nodiscard]] const std::string& message() const { return message_; }

    bool operator==(const Error& other) const {
        return code_ == other.code_ && message_ == other.message_;
    }
    bool operator!=(const Error& other) const { return !(*this == other); }

private:
    ErrorCode   code_ = ErrorCode::Unknown;
    std::string message_;
};

// ── Tag types for in-place construction ─────────────────────
struct OkTag {};
struct ErrTag {};
inline constexpr OkTag  ok_tag{};
inline constexpr ErrTag err_tag{};

// ── Result<T, E> ───────────────────────────────────────────
// A discriminated union that holds either a success value (T) or an error (E).
// Defaults E = Error for convenience.
template<typename T, typename E = Error>
class [[nodiscard]] Result {
public:
    // ── Construction helpers ─────────────────────────────────
    /// Construct a success value.
    static Result ok(T value) { return Result(ok_tag, std::move(value)); }

    /// Construct an error.
    static Result err(E error) { return Result(err_tag, std::move(error)); }

    // ── Observers ────────────────────────────────────────────
    [[nodiscard]] bool is_ok() const { return storage_.index() == 0; }
    [[nodiscard]] bool is_err() const { return storage_.index() == 1; }

    /// Access the success value.  UB if is_err().
    [[nodiscard]] const T& value() const& { return std::get<0>(storage_); }
    [[nodiscard]] T&       value() & { return std::get<0>(storage_); }
    [[nodiscard]] T&&      value() && { return std::get<0>(std::move(storage_)); }

    /// Access the error.  UB if is_ok().
    [[nodiscard]] const E& error() const& { return std::get<1>(storage_); }
    [[nodiscard]] E&       error() & { return std::get<1>(storage_); }
    [[nodiscard]] E&&      error() && { return std::get<1>(std::move(storage_)); }

    /// Return the value if ok, otherwise `fallback`.
    [[nodiscard]] T value_or(T fallback) const& { return is_ok() ? value() : std::move(fallback); }
    [[nodiscard]] T value_or(T fallback) && {
        return is_ok() ? std::move(value()) : std::move(fallback);
    }

    // ── Monadic operations ───────────────────────────────────

    /// Transform the success value (T → U).  Propagates error unchanged.
    template<typename F>
    [[nodiscard]] auto map(F&& func) const& -> Result<std::invoke_result_t<F, const T&>, E> {
        using U = std::invoke_result_t<F, const T&>;
        if (is_ok()) return Result<U, E>::ok(std::invoke(std::forward<F>(func), value()));
        return Result<U, E>::err(error());
    }

    template<typename F>
    [[nodiscard]] auto map(F&& func) && -> Result<std::invoke_result_t<F, T&&>, E> {
        using U = std::invoke_result_t<F, T&&>;
        if (is_ok())
            return Result<U, E>::ok(std::invoke(std::forward<F>(func), std::move(value())));
        return Result<U, E>::err(std::move(error()));
    }

    /// Chain with a function that returns Result<U, E>.  Propagates error.
    /// (flat-map / bind / and_then)
    template<typename F>
    [[nodiscard]] auto and_then(F&& func) const& -> std::invoke_result_t<F, const T&> {
        if (is_ok()) return std::invoke(std::forward<F>(func), value());
        using RetResult = std::invoke_result_t<F, const T&>;
        return RetResult::err(error());
    }

    template<typename F>
    [[nodiscard]] auto and_then(F&& func) && -> std::invoke_result_t<F, T&&> {
        if (is_ok()) return std::invoke(std::forward<F>(func), std::move(value()));
        using RetResult = std::invoke_result_t<F, T&&>;
        return RetResult::err(std::move(error()));
    }

    /// Transform the error (E → E2).  Propagates value unchanged.
    template<typename F>
    [[nodiscard]] auto map_error(F&& func) const& -> Result<T, std::invoke_result_t<F, const E&>> {
        using E2 = std::invoke_result_t<F, const E&>;
        if (is_err()) return Result<T, E2>::err(std::invoke(std::forward<F>(func), error()));
        return Result<T, E2>::ok(value());
    }

    template<typename F>
    [[nodiscard]] auto map_error(F&& func) && -> Result<T, std::invoke_result_t<F, E&&>> {
        using E2 = std::invoke_result_t<F, E&&>;
        if (is_err())
            return Result<T, E2>::err(std::invoke(std::forward<F>(func), std::move(error())));
        return Result<T, E2>::ok(std::move(value()));
    }

private:
    explicit Result(OkTag /*tag*/, T val) : storage_(std::in_place_index<0>, std::move(val)) {}
    explicit Result(ErrTag /*tag*/, E err) : storage_(std::in_place_index<1>, std::move(err)) {}

    std::variant<T, E> storage_;
};

// ── Void specialisation ─────────────────────────────────────
// For operations that succeed or fail but have no return value.
// Uses a Monostate sentinel for the success case.

template<typename E>
class [[nodiscard]] Result<void, E> {
public:
    static Result ok() { return Result(ok_tag); }
    static Result err(E error) { return Result(err_tag, std::move(error)); }

    [[nodiscard]] bool is_ok() const { return storage_.index() == 0; }
    [[nodiscard]] bool is_err() const { return storage_.index() == 1; }

    [[nodiscard]] const E& error() const& { return std::get<1>(storage_); }
    [[nodiscard]] E&       error() & { return std::get<1>(storage_); }
    [[nodiscard]] E&&      error() && { return std::get<1>(std::move(storage_)); }

    /// Chain: run `func()` which returns Result<void, E> only if this is ok.
    template<typename F>
    [[nodiscard]] auto and_then(F&& func) const& -> std::invoke_result_t<F> {
        if (is_ok()) return std::invoke(std::forward<F>(func));
        using RetResult = std::invoke_result_t<F>;
        return RetResult::err(error());
    }

    template<typename F>
    [[nodiscard]] auto and_then(F&& func) && -> std::invoke_result_t<F> {
        if (is_ok()) return std::invoke(std::forward<F>(func));
        using RetResult = std::invoke_result_t<F>;
        return RetResult::err(std::move(error()));
    }

private:
    explicit Result(OkTag /*tag*/) : storage_(std::in_place_index<0>, std::monostate{}) {}
    explicit Result(ErrTag /*tag*/, E e) : storage_(std::in_place_index<1>, std::move(e)) {}

    std::variant<std::monostate, E> storage_;
};

// ── Convenience aliases ─────────────────────────────────────
using VoidResult = Result<void, Error>;

}  // namespace drone::util
