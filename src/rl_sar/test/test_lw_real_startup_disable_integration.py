#!/usr/bin/env python3

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
REAL_SOURCE = ROOT / "src" / "rl_real_LW.cpp"


class RealStartupDisableIntegrationTests(unittest.TestCase):
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
