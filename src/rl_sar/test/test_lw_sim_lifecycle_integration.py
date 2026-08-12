#!/usr/bin/env python3

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SIM_SOURCE = ROOT / "src" / "rl_sim_LW.cpp"
MUJOCO_UTILS = (
    ROOT / "library" / "thirdparty" / "mujoco_simulate" / "mujoco_utils.hpp"
)
SIMULATE_SOURCE = (
    ROOT / "library" / "thirdparty" / "mujoco_simulate" / "simulate.cc"
)


class LWSimLifecycleIntegrationTests(unittest.TestCase):
    def test_constructor_has_no_detached_or_unbounded_startup_wait(self) -> None:
        source = SIM_SOURCE.read_text(encoding="utf-8")
        constructor = source[
            source.index("RL_Real::RL_Real(") : source.index("RL_Real::~RL_Real()")
        ]

        self.assertNotIn(".detach()", constructor)
        self.assertNotIn("while (1)", constructor)
        self.assertLess(
            constructor.index("physics_lifecycle_->Start(filename);"),
            constructor.index("SetupSysJoystick"),
        )

    def test_destructor_joins_physics_after_business_workers(self) -> None:
        source = SIM_SOURCE.read_text(encoding="utf-8")
        destructor = source[
            source.index("RL_Real::~RL_Real()") : source.index(
                "void RL_Real::RequestSimulationStop()"
            )
        ]

        self.assertLess(
            destructor.index("this->loop_control->shutdown();"),
            destructor.index("physics_lifecycle_->Stop();"),
        )
        self.assertLess(
            destructor.index("this->loop_joystick->shutdown();"),
            destructor.index("physics_lifecycle_->Stop();"),
        )

    def test_invalid_scene_is_reported_before_thread_start(self) -> None:
        source = MUJOCO_UTILS.read_text(encoding="utf-8")
        lifecycle = source[
            source.index("class LWMuJoCoPhysicsLifecycle") : source.index(
                "void PhysicsThread("
            )
        ]

        load = lifecycle.index("mjModel* loaded_model = LoadModel")
        diagnostic = lifecycle.index("Failed to load MuJoCo scene")
        worker_start = lifecycle.index("worker_.start(")
        self.assertLess(load, diagnostic)
        self.assertLess(diagnostic, worker_start)

    def test_pending_render_load_is_cancellable(self) -> None:
        source = SIMULATE_SOURCE.read_text(encoding="utf-8")
        load = source[source.index("void Simulate::Load(") : source.index(
            "void Simulate::LoadMessageClear"
        )]
        request_exit = source[
            source.index("void Simulate::RequestExit") : source.index(
                "//------------------------------------- load mjb"
            )
        ]

        self.assertIn("this->exitrequest.load()", load)
        self.assertIn("this->loadrequest = 0;", request_exit)
        self.assertIn("cond_loadrequest.notify_all();", request_exit)

    def test_sigint_never_touches_the_simulation_from_signal_context(self) -> None:
        source = SIM_SOURCE.read_text(encoding="utf-8")
        header = (ROOT / "include" / "rl_sim_LW.hpp").read_text(encoding="utf-8")

        self.assertNotIn("signalHandler", source)
        self.assertNotIn("signal(SIGINT", source)
        self.assertNotIn("RL_Real::instance", source)
        self.assertNotIn("static RL_Real* instance", header)
        self.assertIn("LWSigintWaiter sigint_waiter", source)
        self.assertIn("rclcpp::SignalHandlerOptions::SigTerm", source)

    def test_sigint_is_blocked_before_runtime_objects_and_stops_normally(self) -> None:
        source = SIM_SOURCE.read_text(encoding="utf-8")
        main = source[source.index("int main(") :]

        self.assertLess(
            main.index("LWSigintWaiter sigint_waiter"),
            main.index("rclcpp::init("),
        )
        self.assertLess(
            main.index("rclcpp::init("),
            main.index("std::make_shared<RL_Real>"),
        )
        self.assertIn("shutdown_coordinator.Bind(", main)
        self.assertIn("locked->RequestSimulationStop();", main)
        self.assertIn("shutdown_coordinator.requested()", main)
        self.assertLess(
            main.index("rclcpp::shutdown();"),
            main.index("sigint_waiter.ShutdownAndKeepBlocked();"),
        )
        self.assertLess(
            main.index("sigint_waiter.ShutdownAndKeepBlocked();"),
            main.index("shutdown_coordinator.Unbind();"),
        )

    def test_optional_actuator_models_follow_the_selected_policy_root(self) -> None:
        source = SIM_SOURCE.read_text(encoding="utf-8")
        constructor = source[
            source.index("RL_Real::RL_Real(") : source.index("RL_Real::~RL_Real()")
        ]
        actuator_block = constructor[
            constructor.index("if (this->use_actuator_net_)") : constructor.index(
                "// auto load FSM"
            )
        ]

        self.assertIn(
            "ResolveLWActuatorModelPaths(policy_root_, this->robot_name)",
            actuator_block,
        )
        self.assertIn("LoadLWActuatorModel(actuator_paths.leg)", actuator_block)
        self.assertIn("LoadLWActuatorModel(actuator_paths.foot)", actuator_block)
        self.assertNotIn("POLICY_DIR", actuator_block)
        self.assertIn("Loading leg actuator model:", actuator_block)
        self.assertIn("Loading foot actuator model:", actuator_block)


if __name__ == "__main__":
    unittest.main()
