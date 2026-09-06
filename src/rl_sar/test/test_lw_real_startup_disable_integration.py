#!/usr/bin/env python3

import pathlib
import unittest

from lw_source_checks import cpp_index, cpp_region, require_cpp_order


ROOT = pathlib.Path(__file__).resolve().parents[1]
REAL_SOURCE = ROOT / "src" / "rl_real_LW.cpp"
REAL_HEADER = ROOT / "include" / "rl_real_LW.hpp"
REAL_LAUNCH = ROOT / "launch" / "rl_real_LW.launch.py"
DEBUG_PUBLISHER = ROOT / "library" / "core" / "debug" / "lw_debug_publisher.cpp"


class RealStartupDisableIntegrationTests(unittest.TestCase):
    def test_debug_telemetry_is_rate_bounded_nonblocking_and_source_fresh(self) -> None:
        source = REAL_SOURCE.read_text(encoding="utf-8")
        launch = REAL_LAUNCH.read_text(encoding="utf-8")
        publisher = DEBUG_PUBLISHER.read_text(encoding="utf-8")
        constructor = cpp_region(source, "RL_Real::RL_Real(", "RL_Real::~RL_Real()")

        cpp_index(constructor, 'declare_parameter<std::int64_t>("debug_publish_rate_hz"')
        require_cpp_order(
            constructor,
            "LWDebugPublishPeriod(debug_publish_rate_hz)",
            "LWDebugPublisher::CreateIfEnabled(",
            "this->loop_control->start();",
        )

        cpp_index(publisher, "snapshot_.tryPublish(sequenced_snapshot)")
        with self.assertRaisesRegex(AssertionError, "Missing C\\+\\+ wiring"):
            cpp_index(publisher, "snapshot_.publish(")
        # Freshness/duplicate suppression is exercised by test_lw_debug_publisher.

        self.assertIn(
            "debug_publish_rate_hz = LaunchConfiguration('debug_publish_rate_hz')",
            launch,
        )
        self.assertIn("'debug_publish_rate_hz': debug_publish_rate_hz", launch)
        declaration = launch[launch.index("'debug_publish_rate_hz',") :]
        self.assertIn("default_value='50'", declaration)
        self.assertIn("integer from 1 through 200", declaration)

    def test_fdilink_topics_are_guarded_without_modifying_driver(self) -> None:
        source = REAL_SOURCE.read_text(encoding="utf-8")
        header = REAL_HEADER.read_text(encoding="utf-8")
        launch = REAL_LAUNCH.read_text(encoding="utf-8")

        self.assertIn("SetRemap(src='/imu', dst='/fdilink/raw_imu')", launch)
        self.assertIn(
            "SetRemap(src='/euler_angles', dst='/fdilink/raw_euler')", launch
        )
        self.assertIn('"/fdilink/raw_imu"', source)
        self.assertIn('"/fdilink/raw_euler"', source)
        self.assertIn("LWImuAhrsGuard imu_ahrs_guard_", header)
        with self.assertRaisesRegex(AssertionError, "Missing C\\+\\+ wiring"):
            cpp_index(source, 'create_subscription<sensor_msgs::msg::Imu>("/imu"')

    def test_disable_guard_precedes_ros_and_real_runtime(self) -> None:
        source = REAL_SOURCE.read_text(encoding="utf-8")
        main = source[cpp_index(source, "int main("):]
        require_cpp_order(
            main, "LWDeploymentBundle::Verify(", "if (verify_deployment_only)",
            "LWStartupDisableGuard startup_disable;", "rclcpp::init(argc, argv);",
            "std::make_shared<RL_Real>(",
        )

    def test_constructor_uses_established_guard_before_preload(self) -> None:
        source = REAL_SOURCE.read_text(encoding="utf-8")
        constructor = cpp_region(source, "RL_Real::RL_Real(", "RL_Real::~RL_Real()")

        self.assertNotIn("InitSerial(", constructor)
        require_cpp_order(
            constructor, "startup_disable_->requireHealthy();",
            "this->PreloadModel(policy);", "this->PreloadLWPolicyContext(policy);",
            "startup_disable_->handOffToRuntime(", "this->loop_control->start();",
        )

    def test_command_gate_closes_before_worker_shutdown(self) -> None:
        source = REAL_SOURCE.read_text(encoding="utf-8")
        destructor = cpp_region(source, "RL_Real::~RL_Real()",
                                "void RL_Real::RuntimeDiagnosticsCallback()")
        # Finalization after all workers is owned by the shared lifecycle suite.
        require_cpp_order(
            destructor, "startup_disable_->commandGate().close();",
            "this->loop_control->shutdown();",
        )


if __name__ == "__main__":
    unittest.main()
