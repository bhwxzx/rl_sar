#!/usr/bin/env python3

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SIM_SOURCE = ROOT / "src" / "rl_sim_LW.cpp"
SIM_HEADER = ROOT / "include" / "rl_sim_LW.hpp"
LEGACY_SIM_SOURCE = ROOT / "src" / "rl_sim_mujoco.cpp"
LEGACY_SIM_HEADER = ROOT / "include" / "rl_sim_mujoco.hpp"
MUJOCO_UTILS = (
    ROOT / "library" / "thirdparty" / "mujoco_simulate" / "mujoco_utils.hpp"
)
SIMULATE_SOURCE = (
    ROOT / "library" / "thirdparty" / "mujoco_simulate" / "simulate.cc"
)


class LWSimLifecycleIntegrationTests(unittest.TestCase):
    def test_legacy_detached_mujoco_runtime_was_removed(self) -> None:
        self.assertFalse(LEGACY_SIM_SOURCE.exists())
        self.assertFalse(LEGACY_SIM_HEADER.exists())

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
        self.assertIn("RunLWSimShutdownBoundWorker(", main)
        self.assertIn("rclcpp::spin(rl_sar->ros2_node);", main)
        self.assertIn("shutdown_coordinator.requested()", main)
        self.assertIn("Shutdown requested, exiting Sim2Sim...", main)
        self.assertNotIn("Received SIGINT, exiting Sim2Sim...", main)
        self.assertLess(
            main.index("rclcpp::shutdown();"),
            main.index("sigint_waiter.ShutdownAndKeepBlocked();"),
        )
        self.assertLess(
            main.index("sigint_waiter.ShutdownAndKeepBlocked();"),
            main.index("shutdown_coordinator.Unbind();"),
        )

    def test_actuator_model_runtime_is_absent(self) -> None:
        source = SIM_SOURCE.read_text(encoding="utf-8")
        header = SIM_HEADER.read_text(encoding="utf-8")
        constructor = source[
            source.index("RL_Real::RL_Real(") : source.index("RL_Real::~RL_Real()")
        ]

        for removed in (
            "use_actuator_net_",
            "UpdateActuatorNetwork",
            "LoadLWActuatorModel",
            "actuator_model_paths_",
            "actuator_net_torque_frame_",
            "pos_err_history_",
            "vel_history_",
        ):
            self.assertNotIn(removed, source)
            self.assertNotIn(removed, header)
        self.assertNotIn(".pt", constructor)

    def test_plot_publishing_is_explicit_and_operator_status_is_independent(
        self,
    ) -> None:
        source = SIM_SOURCE.read_text(encoding="utf-8")
        header = SIM_HEADER.read_text(encoding="utf-8")
        constructor = source[
            source.index("RL_Real::RL_Real(") : source.index("RL_Real::~RL_Real()")
        ]
        control = source[
            source.index("void RL_Real::RobotControl()") : source.index(
                "void RL_Real::ApplySimulationControls()"
            )
        ]
        callback = source[
            source.index("void RL_Real::jointstate_plot_callback") : source.index(
                "void RL_Real::OperatorStatusCallback()"
            )
        ]

        self.assertNotIn("#define PLOT", header)
        self.assertNotIn("#ifdef PLOT", source)
        self.assertNotIn("matplotlibcpp", header)
        self.assertNotIn("loop_plot", header)
        self.assertIn(
            "std::unique_ptr<LWSnapshotBuffer<SimDebugSnapshot>> plot_snapshot_",
            header,
        )
        self.assertIn(
            "plot_configuration_(ParseLWSimPlotConfiguration(argc, argv))",
            constructor,
        )
        allocation = constructor.index("plot_snapshot_ =")
        allocation_guard = constructor.rfind(
            "if (plot_configuration_.enabled)", 0, allocation
        )
        self.assertGreaterEqual(allocation_guard, 0)

        operator_timer = constructor.index("this->operator_status_timer_")
        publisher = constructor.index("this->jointstate_plot_publisher_")
        plot_guard = constructor.rfind(
            "if (plot_configuration_.enabled)", 0, publisher
        )
        self.assertLess(operator_timer, plot_guard)
        self.assertLess(plot_guard, publisher)
        self.assertIn('"/LW_joint_states"', constructor[publisher:])
        self.assertIn("LWSimPlotPeriod(plot_configuration_)", constructor[publisher:])

        self.assertIn("if (plot_snapshot_)", control)
        self.assertIn("hooks.after_command_delivery =", control)
        self.assertIn("plot_snapshot_->publish(", control)
        self.assertNotIn("debug_snapshot_", source)

        self.assertIn("plot_snapshot_->read(snapshot)", callback)
        self.assertIn("jointstate_plot_publisher_->publish(msg)", callback)
        for payload_name in (
            "right_hip_now",
            "right_hip_target",
            "l_foot_x",
            "cmd_vel_x",
            "base_q_w",
        ):
            self.assertIn(payload_name, callback)

    def test_pd_ff_mujoco_torques_are_validated_transactionally(self) -> None:
        source = SIM_SOURCE.read_text(encoding="utf-8")
        command = source[
            source.index("void RL_Real::SetCommand(") : source.index(
                "void RL_Real::SetupSysJoystick("
            )
        ]

        self.assertIn("command->motor_command.tau[i]", command)
        self.assertIn("command->motor_command.kp[i]", command)
        self.assertIn("command->motor_command.kd[i]", command)
        self.assertNotIn("actuator_net", command)

        prepare = command.index("PrepareLWSimTorques(")
        first_write = command.index("mj_data->ctrl[")
        self.assertLess(prepare, first_write)
        self.assertIn("if (!validation.valid())", command[prepare:first_write])
        self.assertIn(
            "LWSafetyEvent::SimulationActuatorCommandInvalid",
            command[prepare:first_write],
        )
        self.assertNotIn("mj_data->ctrl[", command[:prepare])


if __name__ == "__main__":
    unittest.main()
