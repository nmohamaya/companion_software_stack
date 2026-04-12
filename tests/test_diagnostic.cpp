// tests/test_diagnostic.cpp
// Unit tests for FrameDiagnostics — the pipeline-level error/metric collector.
//
// Tests:
//   1. Empty diagnostics has no errors/warnings
//   2. add_error increments error count
//   3. add_warning increments warning count
//   4. add_fatal increments both fatal and error count
//   5. add_metric records value
//   6. add_timing records timing entry
//   7. worst_severity reflects highest level
//   8. merge combines entries from another diagnostics
//   9. reset clears all state
//  10. ScopedDiagTimer records elapsed time

#include "util/diagnostic.h"

#include <thread>

#include <gtest/gtest.h>

using namespace drone::util;

TEST(DiagnosticTest, EmptyHasNoErrorsOrWarnings) {
    FrameDiagnostics diag(1);
    EXPECT_FALSE(diag.has_errors());
    EXPECT_FALSE(diag.has_warnings());
    EXPECT_FALSE(diag.has_fatal());
    EXPECT_EQ(diag.error_count(), 0);
    EXPECT_EQ(diag.warning_count(), 0);
    EXPECT_EQ(diag.frame_id(), 1u);
    EXPECT_TRUE(diag.entries().empty());
}

TEST(DiagnosticTest, AddErrorIncrementsCount) {
    FrameDiagnostics diag(1);
    diag.add_error("TestComp", "Something broke");
    EXPECT_TRUE(diag.has_errors());
    EXPECT_EQ(diag.error_count(), 1);
    EXPECT_FALSE(diag.has_warnings());
}

TEST(DiagnosticTest, AddWarningIncrementsCount) {
    FrameDiagnostics diag(1);
    diag.add_warning("TestComp", "Something degraded");
    EXPECT_TRUE(diag.has_warnings());
    EXPECT_EQ(diag.warning_count(), 1);
    EXPECT_FALSE(diag.has_errors());
}

TEST(DiagnosticTest, AddFatalIncrementsBothCounts) {
    FrameDiagnostics diag(1);
    diag.add_fatal("TestComp", "Unrecoverable failure");
    EXPECT_TRUE(diag.has_fatal());
    EXPECT_TRUE(diag.has_errors());
    EXPECT_EQ(diag.error_count(), 1);
}

TEST(DiagnosticTest, AddMetricRecordsValue) {
    FrameDiagnostics diag(1);
    diag.add_metric("Extractor", "num_features", 127.0);

    ASSERT_EQ(diag.entries().size(), 1u);
    EXPECT_EQ(diag.entries()[0].component, "Extractor");
    EXPECT_EQ(diag.entries()[0].message, "num_features");
    EXPECT_DOUBLE_EQ(diag.entries()[0].value, 127.0);
    EXPECT_EQ(diag.entries()[0].severity, DiagSeverity::INFO);
}

TEST(DiagnosticTest, AddTimingRecordsEntry) {
    FrameDiagnostics diag(1);
    diag.add_timing("Matcher", 4.2);

    ASSERT_EQ(diag.entries().size(), 1u);
    EXPECT_EQ(diag.entries()[0].component, "Matcher");
    EXPECT_EQ(diag.entries()[0].message, "timing_ms");
    EXPECT_DOUBLE_EQ(diag.entries()[0].value, 4.2);
}

TEST(DiagnosticTest, WorstSeverity) {
    FrameDiagnostics diag(1);
    EXPECT_EQ(diag.worst_severity(), DiagSeverity::INFO);

    diag.add_warning("A", "warn");
    EXPECT_EQ(diag.worst_severity(), DiagSeverity::WARN);

    diag.add_error("B", "err");
    EXPECT_EQ(diag.worst_severity(), DiagSeverity::ERROR);

    diag.add_fatal("C", "fatal");
    EXPECT_EQ(diag.worst_severity(), DiagSeverity::FATAL);
}

TEST(DiagnosticTest, MergeCombinesEntries) {
    FrameDiagnostics diag1(1);
    diag1.add_error("A", "err1");
    diag1.add_metric("A", "x", 1.0);

    FrameDiagnostics diag2(1);
    diag2.add_warning("B", "warn1");
    diag2.add_metric("B", "y", 2.0);

    diag1.merge(diag2);
    EXPECT_EQ(diag1.entries().size(), 4u);
    EXPECT_EQ(diag1.error_count(), 1);
    EXPECT_EQ(diag1.warning_count(), 1);
}

TEST(DiagnosticTest, ResetClearsState) {
    FrameDiagnostics diag(1);
    diag.add_error("A", "err");
    diag.add_warning("B", "warn");
    diag.add_metric("C", "x", 1.0);

    diag.reset(42);
    EXPECT_EQ(diag.frame_id(), 42u);
    EXPECT_FALSE(diag.has_errors());
    EXPECT_FALSE(diag.has_warnings());
    EXPECT_TRUE(diag.entries().empty());
    EXPECT_EQ(diag.error_count(), 0);
    EXPECT_EQ(diag.warning_count(), 0);
}

TEST(DiagnosticTest, ScopedTimerRecordsElapsed) {
    FrameDiagnostics diag(1);
    {
        ScopedDiagTimer timer(diag, "SlowOp");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    ASSERT_GE(diag.entries().size(), 1u);
    bool found = false;
    for (const auto& e : diag.entries()) {
        if (e.component == "SlowOp" && e.message == "timing_ms") {
            EXPECT_GT(e.value, 0.0);
            found = true;
        }
    }
    EXPECT_TRUE(found) << "ScopedDiagTimer should record a timing entry";
}

TEST(DiagnosticTest, LogSummaryDoesNotThrow) {
    FrameDiagnostics diag(99);
    diag.add_metric("A", "x", 1.0);
    diag.add_warning("B", "degraded");
    diag.add_error("C", "failed");
    // Should not throw
    EXPECT_NO_THROW(diag.log_summary("TestPipeline"));
}

TEST(DiagnosticTest, SeverityStrings) {
    EXPECT_STREQ(diag_severity_str(DiagSeverity::INFO), "INFO");
    EXPECT_STREQ(diag_severity_str(DiagSeverity::WARN), "WARN");
    EXPECT_STREQ(diag_severity_str(DiagSeverity::ERROR), "ERROR");
    EXPECT_STREQ(diag_severity_str(DiagSeverity::FATAL), "FATAL");
}
