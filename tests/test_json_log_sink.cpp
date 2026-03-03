// tests/test_json_log_sink.cpp
// Unit tests for the structured JSON logging sink.
#include "util/json_log_sink.h"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

using drone::util::JsonLogSink;
using drone::util::JsonLogSink_mt;
using drone::util::JsonLogSink_st;

// ── Helper: create a sink that writes to a temp file ────────
class JsonSinkTest : public ::testing::Test {
protected:
    void SetUp() override {
        tmp_path_ = "/tmp/test_json_sink_" + std::to_string(getpid()) + ".jsonl";
        fp_       = std::fopen(tmp_path_.c_str(), "w");
        ASSERT_NE(fp_, nullptr);

        sink_   = std::make_shared<JsonLogSink_st>(fp_);
        logger_ = std::make_shared<spdlog::logger>("test_json", sink_);
        logger_->set_level(spdlog::level::trace);
    }

    void TearDown() override {
        logger_.reset();
        sink_.reset();
        if (fp_) std::fclose(fp_);
        std::remove(tmp_path_.c_str());
    }

    /// Read the temp file contents.
    std::string read_output() {
        std::fflush(fp_);
        std::ifstream ifs(tmp_path_);
        return {std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>()};
    }

    /// Get the last JSON line from the sink (via accessor).
    std::string last_json() { return sink_->last_json(); }

    std::string                     tmp_path_;
    std::FILE*                      fp_ = nullptr;
    std::shared_ptr<JsonLogSink_st> sink_;
    std::shared_ptr<spdlog::logger> logger_;
};

// ── Basic structure ─────────────────────────────────────────

TEST_F(JsonSinkTest, OutputIsValidJsonLine) {
    logger_->info("hello world");
    auto json = last_json();

    // Must start with { and end with }\n
    ASSERT_FALSE(json.empty());
    EXPECT_EQ(json.front(), '{');
    EXPECT_EQ(json.back(), '\n');
    EXPECT_EQ(json[json.size() - 2], '}');
}

TEST_F(JsonSinkTest, ContainsRequiredFields) {
    logger_->info("test message");
    auto json = last_json();

    EXPECT_NE(json.find("\"ts\":"), std::string::npos);
    EXPECT_NE(json.find("\"level\":"), std::string::npos);
    EXPECT_NE(json.find("\"logger\":"), std::string::npos);
    EXPECT_NE(json.find("\"thread\":"), std::string::npos);
    EXPECT_NE(json.find("\"pid\":"), std::string::npos);
    EXPECT_NE(json.find("\"msg\":"), std::string::npos);
}

TEST_F(JsonSinkTest, MessageContentPreserved) {
    logger_->info("specific test payload 42");
    auto json = last_json();

    EXPECT_NE(json.find("specific test payload 42"), std::string::npos);
}

// ── Level strings ───────────────────────────────────────────

TEST_F(JsonSinkTest, LevelTrace) {
    logger_->trace("t");
    EXPECT_NE(last_json().find("\"level\":\"trace\""), std::string::npos);
}

TEST_F(JsonSinkTest, LevelDebug) {
    logger_->debug("d");
    EXPECT_NE(last_json().find("\"level\":\"debug\""), std::string::npos);
}

TEST_F(JsonSinkTest, LevelInfo) {
    logger_->info("i");
    EXPECT_NE(last_json().find("\"level\":\"info\""), std::string::npos);
}

TEST_F(JsonSinkTest, LevelWarn) {
    logger_->warn("w");
    EXPECT_NE(last_json().find("\"level\":\"warn\""), std::string::npos);
}

TEST_F(JsonSinkTest, LevelError) {
    logger_->error("e");
    EXPECT_NE(last_json().find("\"level\":\"error\""), std::string::npos);
}

TEST_F(JsonSinkTest, LevelCritical) {
    logger_->critical("c");
    EXPECT_NE(last_json().find("\"level\":\"critical\""), std::string::npos);
}

// ── Logger name ─────────────────────────────────────────────

TEST_F(JsonSinkTest, LoggerNameIncluded) {
    logger_->info("x");
    EXPECT_NE(last_json().find("\"logger\":\"test_json\""), std::string::npos);
}

// ── Timestamp format ────────────────────────────────────────

TEST_F(JsonSinkTest, TimestampIsISO8601) {
    logger_->info("ts check");
    auto json = last_json();

    // Find the timestamp value: "ts":"YYYY-MM-DDTHH:MM:SS.uuuuuu"
    auto pos = json.find("\"ts\":\"");
    ASSERT_NE(pos, std::string::npos);
    pos += 6;  // skip past "ts":"
    auto end = json.find('\"', pos);
    ASSERT_NE(end, std::string::npos);
    std::string ts = json.substr(pos, end - pos);

    // Format: YYYY-MM-DDTHH:MM:SS.uuuuuu (26 chars)
    EXPECT_EQ(ts.size(), 26u);
    EXPECT_EQ(ts[4], '-');
    EXPECT_EQ(ts[7], '-');
    EXPECT_EQ(ts[10], 'T');
    EXPECT_EQ(ts[13], ':');
    EXPECT_EQ(ts[16], ':');
    EXPECT_EQ(ts[19], '.');
}

// ── Thread ID ───────────────────────────────────────────────

TEST_F(JsonSinkTest, ThreadIdIsNumeric) {
    logger_->info("tid");
    auto json = last_json();

    auto pos = json.find("\"thread\":");
    ASSERT_NE(pos, std::string::npos);
    pos += 9;  // skip past "thread":
    // Must be a digit
    EXPECT_TRUE(std::isdigit(json[pos])) << "thread value should start with digit";
}

// ── PID ─────────────────────────────────────────────────────

TEST_F(JsonSinkTest, PidMatchesCurrentProcess) {
    logger_->info("pid check");
    auto json   = last_json();
    auto needle = "\"pid\":" + std::to_string(static_cast<int>(getpid()));
    EXPECT_NE(json.find(needle), std::string::npos);
}

// ── JSON escaping ───────────────────────────────────────────

TEST_F(JsonSinkTest, EscapesDoubleQuotes) {
    logger_->info("say \"hello\"");
    auto json = last_json();
    EXPECT_NE(json.find("say \\\"hello\\\""), std::string::npos);
}

TEST_F(JsonSinkTest, EscapesBackslash) {
    logger_->info("path\\to\\file");
    auto json = last_json();
    EXPECT_NE(json.find("path\\\\to\\\\file"), std::string::npos);
}

TEST_F(JsonSinkTest, EscapesNewline) {
    logger_->info("line1\nline2");
    auto json = last_json();
    EXPECT_NE(json.find("line1\\nline2"), std::string::npos);
    // Ensure the JSON itself is a single line (no raw newlines in msg)
    auto first_newline = json.find('\n');
    EXPECT_EQ(first_newline, json.size() - 1) << "JSON line must be single-line";
}

TEST_F(JsonSinkTest, EscapesTab) {
    logger_->info("col1\tcol2");
    auto json = last_json();
    EXPECT_NE(json.find("col1\\tcol2"), std::string::npos);
}

TEST_F(JsonSinkTest, EscapesCarriageReturn) {
    logger_->info("before\rafter");
    auto json = last_json();
    EXPECT_NE(json.find("before\\rafter"), std::string::npos);
}

TEST_F(JsonSinkTest, EscapesControlCharacters) {
    std::string msg = "ctrl-";
    msg += '\x01';
    msg += "-char";
    logger_->info(msg);
    auto json = last_json();
    EXPECT_NE(json.find("\\u0001"), std::string::npos);
}

// ── File output ─────────────────────────────────────────────

TEST_F(JsonSinkTest, WritesToFile) {
    logger_->info("file output test");
    auto content = read_output();
    EXPECT_NE(content.find("file output test"), std::string::npos);
}

TEST_F(JsonSinkTest, MultipleMessagesProduceMultipleLines) {
    logger_->info("msg1");
    logger_->warn("msg2");
    logger_->error("msg3");
    auto content = read_output();

    // Count newlines
    int lines = 0;
    for (char c : content) {
        if (c == '\n') ++lines;
    }
    EXPECT_EQ(lines, 3);
}

// ── Empty message ───────────────────────────────────────────

TEST_F(JsonSinkTest, EmptyMessageProducesValidJson) {
    logger_->info("");
    auto json = last_json();
    EXPECT_NE(json.find("\"msg\":\"\""), std::string::npos);
    EXPECT_EQ(json.front(), '{');
}

// ── Long message ────────────────────────────────────────────

TEST_F(JsonSinkTest, LongMessageHandled) {
    std::string long_msg(2048, 'X');
    logger_->info(long_msg);
    auto json = last_json();
    EXPECT_NE(json.find(std::string(100, 'X')), std::string::npos);
    EXPECT_EQ(json.front(), '{');
    EXPECT_EQ(json.back(), '\n');
}

// ── Thread-safe variant compiles and works ──────────────────

TEST(JsonSinkMtTest, MtVariantProducesOutput) {
    std::FILE* tmp = std::tmpfile();
    ASSERT_NE(tmp, nullptr);
    {
        auto sink   = std::make_shared<JsonLogSink_mt>(tmp);
        auto logger = std::make_shared<spdlog::logger>("mt_test", sink);
        logger->set_level(spdlog::level::info);

        // Just verify it compiles and doesn't crash
        logger->info("mt test");
        EXPECT_FALSE(sink->last_json().empty());
    }
    std::fclose(tmp);
}

// ── detail::json_escape unit tests ──────────────────────────

TEST(JsonEscapeTest, PlainStringUnchanged) {
    std::string out;
    drone::util::detail::json_escape(out, "hello world 123");
    EXPECT_EQ(out, "hello world 123");
}

TEST(JsonEscapeTest, AllSpecialChars) {
    std::string out;
    drone::util::detail::json_escape(out, "\"\\\n\r\t");
    EXPECT_EQ(out, "\\\"\\\\\\n\\r\\t");
}

TEST(JsonEscapeTest, EmptyString) {
    std::string out;
    drone::util::detail::json_escape(out, "");
    EXPECT_TRUE(out.empty());
}

TEST(JsonEscapeTest, UnicodePassthrough) {
    // UTF-8 multi-byte characters should pass through unchanged
    std::string out;
    drone::util::detail::json_escape(out, "café ñ 日本語");
    EXPECT_EQ(out, "café ñ 日本語");
}

// ── detail::format_timestamp ────────────────────────────────

TEST(FormatTimestampTest, ProducesCorrectLength) {
    auto ts = drone::util::detail::format_timestamp(std::chrono::system_clock::now());
    EXPECT_EQ(ts.size(), 26u);
}

TEST(FormatTimestampTest, ContainsTSeparator) {
    auto ts = drone::util::detail::format_timestamp(std::chrono::system_clock::now());
    EXPECT_EQ(ts[10], 'T');
}

// ── detail::level_to_str ────────────────────────────────────

TEST(LevelToStrTest, AllLevels) {
    EXPECT_STREQ(drone::util::detail::level_to_str(spdlog::level::trace), "trace");
    EXPECT_STREQ(drone::util::detail::level_to_str(spdlog::level::debug), "debug");
    EXPECT_STREQ(drone::util::detail::level_to_str(spdlog::level::info), "info");
    EXPECT_STREQ(drone::util::detail::level_to_str(spdlog::level::warn), "warn");
    EXPECT_STREQ(drone::util::detail::level_to_str(spdlog::level::err), "error");
    EXPECT_STREQ(drone::util::detail::level_to_str(spdlog::level::critical), "critical");
}

// ── Integration: LogConfig with json_mode ───────────────────

#include "util/log_config.h"

TEST(LogConfigJsonTest, JsonModeInitDoesNotCrash) {
    // Init with json_mode = true to a temp dir
    std::string tmp_dir = "/tmp/test_logconfig_json_" + std::to_string(getpid());
    std::filesystem::create_directories(tmp_dir);

    // Should not throw
    EXPECT_NO_THROW(LogConfig::init("json_test", tmp_dir, "debug", true));

    // Logger should be set
    auto logger = spdlog::default_logger();
    EXPECT_EQ(logger->name(), "json_test");

    // Clean up
    spdlog::drop_all();
    std::filesystem::remove_all(tmp_dir);
}

TEST(LogConfigJsonTest, HumanModeStillWorks) {
    std::string tmp_dir = "/tmp/test_logconfig_human_" + std::to_string(getpid());
    std::filesystem::create_directories(tmp_dir);

    EXPECT_NO_THROW(LogConfig::init("human_test", tmp_dir, "info", false));

    auto logger = spdlog::default_logger();
    EXPECT_EQ(logger->name(), "human_test");

    spdlog::drop_all();
    std::filesystem::remove_all(tmp_dir);
}

// ── ParsedArgs --json-logs flag ─────────────────────────────

#include "util/arg_parser.h"

TEST(ArgParserJsonTest, DefaultJsonLogsFalse) {
    ParsedArgs args;
    EXPECT_FALSE(args.json_logs);
}

TEST(ArgParserJsonTest, JsonLogsFlagParsed) {
    const char* argv[] = {"test", "--json-logs"};
    auto        args   = parse_args(2, const_cast<char**>(argv), "test");
    EXPECT_TRUE(args.json_logs);
}

TEST(ArgParserJsonTest, JsonLogsWithOtherFlags) {
    const char* argv[] = {"test", "--log-level", "debug", "--json-logs", "--sim"};
    auto        args   = parse_args(5, const_cast<char**>(argv), "test");
    EXPECT_TRUE(args.json_logs);
    EXPECT_TRUE(args.simulation);
    EXPECT_EQ(args.log_level, "debug");
}
