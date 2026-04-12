"""Tests for orchestrator.commands.boundaries — agent boundary enforcement."""

from __future__ import annotations

import pytest

from orchestrator.commands.boundaries import check_boundaries, _matches_any


class TestMatchesAny:
    """Test the pattern matching helper."""

    def test_directory_prefix(self):
        assert _matches_any("docs/foo.md", ["docs/"])

    def test_directory_prefix_no_match(self):
        assert not _matches_any("src/docs.txt", ["docs/"])

    def test_glob_pattern(self):
        assert _matches_any("tests/test_perception_unit.cpp", ["tests/test_*perception*"])

    def test_glob_no_match(self):
        assert not _matches_any("tests/test_nav.cpp", ["tests/test_*perception*"])

    def test_exact_match(self):
        assert _matches_any("CLAUDE.md", ["CLAUDE.md"])

    def test_exact_no_match(self):
        assert not _matches_any("README.md", ["CLAUDE.md"])

    def test_nested_glob(self):
        assert _matches_any(
            "common/hal/include/hal/camera_interface.h",
            ["common/hal/include/hal/*camera*"],
        )


class TestCheckBoundariesPerception:
    """Test boundary checks for feature-perception role."""

    def test_allowed_p1(self):
        assert check_boundaries(
            ["process1_video_capture/src/main.cpp"], "feature-perception"
        ) == []

    def test_allowed_p2(self):
        assert check_boundaries(
            ["process2_perception/src/detector.cpp"], "feature-perception"
        ) == []

    def test_allowed_tests(self):
        assert check_boundaries(
            ["tests/test_perception_pipeline.cpp"], "feature-perception"
        ) == []

    def test_allowed_hal_camera(self):
        assert check_boundaries(
            ["common/hal/include/hal/camera_interface.h"], "feature-perception"
        ) == []

    def test_violation_p4(self):
        violations = check_boundaries(
            ["process4_mission_planner/src/main.cpp"], "feature-perception"
        )
        assert len(violations) == 1

    def test_always_allowed_docs(self):
        assert check_boundaries(
            ["docs/design/perception_design.md"], "feature-perception"
        ) == []

    def test_always_allowed_tasks(self):
        assert check_boundaries(
            ["tasks/active-work.md"], "feature-perception"
        ) == []


class TestCheckBoundariesNav:
    """Test boundary checks for feature-nav role."""

    def test_allowed_p3(self):
        assert check_boundaries(
            ["process3_slam_vio_nav/src/slam.cpp"], "feature-nav"
        ) == []

    def test_allowed_p4(self):
        assert check_boundaries(
            ["process4_mission_planner/src/fsm.cpp"], "feature-nav"
        ) == []

    def test_allowed_planner_hal(self):
        assert check_boundaries(
            ["common/hal/include/hal/path_planner.h"], "feature-nav"
        ) == []

    def test_violation_p2(self):
        violations = check_boundaries(
            ["process2_perception/src/tracker.cpp"], "feature-nav"
        )
        assert len(violations) == 1


class TestCheckBoundariesIntegration:
    """Test boundary checks for feature-integration role."""

    def test_allowed_p5(self):
        assert check_boundaries(
            ["process5_comms/src/fc_link.cpp"], "feature-integration"
        ) == []

    def test_allowed_p6(self):
        assert check_boundaries(
            ["process6_payload_manager/src/gimbal.cpp"], "feature-integration"
        ) == []

    def test_allowed_p7(self):
        assert check_boundaries(
            ["process7_system_monitor/src/health.cpp"], "feature-integration"
        ) == []

    def test_allowed_ipc(self):
        assert check_boundaries(
            ["common/ipc/src/zenoh_bus.cpp"], "feature-integration"
        ) == []

    def test_violation_p3(self):
        violations = check_boundaries(
            ["process3_slam_vio_nav/src/vio.cpp"], "feature-integration"
        )
        assert len(violations) == 1


class TestCheckBoundariesInfraCore:
    """Test boundary checks for feature-infra-core role."""

    def test_allowed_util(self):
        assert check_boundaries(
            ["common/util/src/config.cpp"], "feature-infra-core"
        ) == []

    def test_allowed_cmake(self):
        assert check_boundaries(
            ["CMakeLists.txt"], "feature-infra-core"
        ) == []

    def test_allowed_config(self):
        assert check_boundaries(
            ["config/default.json"], "feature-infra-core"
        ) == []

    def test_violation_p5(self):
        violations = check_boundaries(
            ["process5_comms/src/fc_link.cpp"], "feature-infra-core"
        )
        assert len(violations) == 1


class TestCheckBoundariesInfraPlatform:
    """Test boundary checks for feature-infra-platform role."""

    def test_allowed_deploy(self):
        assert check_boundaries(
            ["deploy/build.sh"], "feature-infra-platform"
        ) == []

    def test_allowed_scripts(self):
        assert check_boundaries(
            ["scripts/start-agent.sh"], "feature-infra-platform"
        ) == []

    def test_allowed_github(self):
        assert check_boundaries(
            [".github/workflows/ci.yml"], "feature-infra-platform"
        ) == []

    def test_violation_common_util(self):
        violations = check_boundaries(
            ["common/util/src/config.cpp"], "feature-infra-platform"
        )
        assert len(violations) == 1


class TestMixedFiles:
    """Test with multiple files, some allowed, some not."""

    def test_mixed_results(self):
        files = [
            "process1_video_capture/src/main.cpp",  # allowed for perception
            "process4_mission_planner/src/fsm.cpp",  # violation for perception
            "docs/tracking/PROGRESS.md",              # always allowed
            "tasks/active-work.md",                   # always allowed
        ]
        violations = check_boundaries(files, "feature-perception")
        assert violations == ["process4_mission_planner/src/fsm.cpp"]

    def test_unknown_role_all_violations(self):
        files = ["process1_video_capture/src/main.cpp"]
        # Unknown role has no role-specific patterns
        violations = check_boundaries(files, "nonexistent-role")
        assert len(violations) == 1

    def test_docs_always_allowed_for_any_role(self):
        files = ["docs/anything.md"]
        for role in [
            "feature-perception", "feature-nav", "feature-integration",
            "feature-infra-core", "feature-infra-platform",
        ]:
            assert check_boundaries(files, role) == []
