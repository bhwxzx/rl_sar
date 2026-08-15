#!/usr/bin/env python3

import pathlib
import unittest


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
        constructor = source[
            source.index("RL_Real::RL_Real(") : source.index("RL_Real::~RL_Real()")
        ]

        self.assertIn(
            'declare_parameter<std::int64_t>(\n'
            '            "debug_publish_rate_hz"',
            constructor,
        )
        validation = constructor.index("LWDebugPublishPeriod(debug_publish_rate_hz)")
        creation = constructor.index("LWDebugPublisher::CreateIfEnabled(")
        worker_start = constructor.index("this->loop_control->start();")
        self.assertLess(validation, creation)
        self.assertLess(creation, worker_start)

        self.assertIn("snapshot_.tryPublish(sequenced_snapshot)", publisher)
        self.assertNotIn("snapshot_.publish(", publisher)
        self.assertIn(
            "sequenced_snapshot.sequence <= last_published_sequence_", publisher
        )

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
        self.assertNotIn('create_subscription<sensor_msgs::msg::Imu>(\n        "/imu"', source)

    def test_disable_guard_precedes_ros_and_real_runtime(self) -> None:
        source = REAL_SOURCE.read_text(encoding="utf-8")
        main = source[source.index("int main(") :]

        verify = main.index("LWDeploymentBundle::Verify(")
        verification_only = main.index("if (verify_deployment_only)")
        startup_disable = main.index("LWStartupDisableGuard startup_disable;")
        ros_init = main.index("rclcpp::init(argc, argv);")
        real_runtime = main.index("std::make_shared<RL_Real>(")

        self.assertLess(verify, verification_only)
        self.assertLess(verification_only, startup_disable)
        self.assertLess(startup_disable, ros_init)
        self.assertLess(ros_init, real_runtime)

    def test_constructor_uses_established_guard_before_preload(self) -> None:
        source = REAL_SOURCE.read_text(encoding="utf-8")
        constructor = source[
            source.index("RL_Real::RL_Real(") : source.index("RL_Real::~RL_Real()")
        ]

        self.assertNotIn("InitSerial(", constructor)
        self.assertLess(
            constructor.index("startup_disable_->requireHealthy();"),
            constructor.index("this->PreloadModel(policy);"),
        )
        self.assertLess(
            constructor.index("startup_disable_->handOffToRuntime("),
            constructor.index("this->loop_control->start();"),
        )

    def test_normal_destruction_finalizes_after_worker_shutdown(self) -> None:
        source = REAL_SOURCE.read_text(encoding="utf-8")
        destructor = source[
            source.index("RL_Real::~RL_Real()") : source.index(
                "void RL_Real::RuntimeDiagnosticsCallback()"
            )
        ]

        self.assertLess(
            destructor.index("startup_disable_->commandGate().close();"),
            destructor.index("this->loop_control->shutdown();"),
        )
        self.assertLess(
            destructor.index("this->loop_joystick->shutdown();"),
            destructor.index("startup_disable_->finalize();"),
        )


if __name__ == "__main__":
    unittest.main()
