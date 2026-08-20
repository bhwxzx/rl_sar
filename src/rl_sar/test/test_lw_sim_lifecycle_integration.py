#!/usr/bin/env python3

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SIM_SOURCE = ROOT / "src" / "rl_sim_LW.cpp"
REAL_SOURCE = ROOT / "src" / "rl_real_LW.cpp"
SIM_HEADER = ROOT / "include" / "rl_sim_LW.hpp"
SIM_DEBUG_HEADER = (
    ROOT / "library" / "core" / "simulation" / "lw_sim_debug_message.hpp"
)
SIM_DEBUG_SOURCE = (
    ROOT / "library" / "core" / "simulation" / "lw_sim_debug_message.cpp"
)
MUJOCO_ADAPTER_SOURCE = (
    ROOT / "library" / "core" / "simulation" / "lw_mujoco_control_adapter.cpp"
)
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
            constructor.index("ValidateLWBaseConfiguration("),
            constructor.index("physics_lifecycle_->Start("),
        )
        self.assertLess(
            constructor.index("physics_lifecycle_->Start("),
            constructor.index("SetupSysJoystick"),
        )
        self.assertIn("LWMuJoCoControlAdapter", constructor)
        self.assertIn("runtime_configuration.joint_names", constructor)
        self.assertIn("runtime_configuration.joint_mapping", constructor)

    def test_real_and_sim_finish_fallible_setup_before_worker_start(self) -> None:
        constructors = {}
        for name, path in (("real", REAL_SOURCE), ("sim", SIM_SOURCE)):
            source = path.read_text(encoding="utf-8")
            constructors[name] = source[
                source.index("RL_Real::RL_Real(") : source.index(
                    "RL_Real::~RL_Real()"
                )
            ]

        setup_markers = {
            "real": (
                "runtime_diagnostics_timer_ = ros2_node->create_wall_timer(",
                "LWDebugPublisher::CreateIfEnabled(",
                "this->CSVInit(this->robot_name);",
                "startup_disable_->handOffToRuntime(",
            ),
            "sim": (
                "operator_status_timer_ = ros2_node->create_wall_timer(",
                "jointstate_plot_publisher_ =",
                "plot_timer_ = ros2_node->create_wall_timer(",
                "this->CSVInit(this->robot_name);",
            ),
        }

        for name, constructor in constructors.items():
            starts = [
                constructor.index("this->loop_joystick->start();"),
                constructor.index("this->loop_rl->start();"),
                constructor.index("this->loop_control->start();"),
            ]
            self.assertEqual(
                starts,
                sorted(starts),
                f"{name} worker startup order drifted",
            )
            first_start = starts[0]
            for marker in setup_markers[name]:
                marker_position = constructor.index(marker)
                self.assertLess(
                    marker_position,
                    first_start,
                    f"{name} performs fallible setup after worker startup: {marker}",
                )

            startup_tail = constructor[first_start:]
            for marker in (
                "create_wall_timer(",
                "create_publisher<",
                "CreateIfEnabled(",
                "CSVInit(",
            ):
                self.assertNotIn(
                    marker,
                    startup_tail,
                    f"{name} added fallible resource setup after worker startup",
                )

            shutdowns = [
                startup_tail.index("this->loop_control->shutdown();"),
                startup_tail.index("this->loop_rl->shutdown();"),
                startup_tail.index("this->loop_joystick->shutdown();"),
            ]
            self.assertEqual(
                shutdowns,
                sorted(shutdowns),
                f"{name} startup rollback order drifted",
            )

    def test_real_and_sim_join_workers_before_backend_shutdown(self) -> None:
        real_source = REAL_SOURCE.read_text(encoding="utf-8")
        sim_source = SIM_SOURCE.read_text(encoding="utf-8")
        real_destructor = real_source[
            real_source.index("RL_Real::~RL_Real()") : real_source.index(
                "void RL_Real::RuntimeDiagnosticsCallback()"
            )
        ]
        sim_destructor = sim_source[
            sim_source.index("RL_Real::~RL_Real()") : sim_source.index(
                "void RL_Real::RequestSimulationStop()"
            )
        ]

        for name, destructor in (
            ("real", real_destructor),
            ("sim", sim_destructor),
        ):
            shutdowns = [
                destructor.index("this->loop_control->shutdown();"),
                destructor.index("this->loop_rl->shutdown();"),
                destructor.index("this->loop_joystick->shutdown();"),
            ]
            self.assertEqual(
                shutdowns,
                sorted(shutdowns),
                f"{name} destructor worker shutdown order drifted",
            )

        self.assertLess(
            real_destructor.index("this->loop_joystick->shutdown();"),
            real_destructor.index("startup_disable_->finalize();"),
        )
        self.assertLess(
            sim_destructor.index("this->loop_joystick->shutdown();"),
            sim_destructor.index("physics_lifecycle_->Stop();"),
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
        validation = lifecycle.index("validate_before_start(*loaded_model")
        worker_start = lifecycle.index("worker_.start(")
        self.assertLess(load, diagnostic)
        self.assertLess(diagnostic, worker_start)
        self.assertLess(validation, worker_start)

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
        debug_header = SIM_DEBUG_HEADER.read_text(encoding="utf-8")
        debug_source = SIM_DEBUG_SOURCE.read_text(encoding="utf-8")
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
            "std::unique_ptr<LWSimDebugMessageCache> plot_message_cache_",
            header,
        )
        self.assertIn(
            "std::unique_ptr<SimDebugSnapshot> plot_write_snapshot_",
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
        self.assertIn("plot_write_snapshot_->robot_state = robot_state", control)
        self.assertIn("plot_write_snapshot_->robot_command = robot_command", control)
        self.assertIn("plot_snapshot_->publish(*plot_write_snapshot_)", control)
        self.assertNotIn("SimDebugSnapshot{", control)
        self.assertNotIn("debug_snapshot_", source)

        self.assertIn("plot_snapshot_->read(*plot_read_snapshot_)", callback)
        self.assertIn("plot_message_cache_->Populate(", callback)
        self.assertIn("jointstate_plot_publisher_->publish(message)", callback)
        self.assertNotIn("params.Get", callback)
        self.assertNotIn("mj_name2id", callback)
        self.assertNotIn("resize(", callback)
        for payload_name in (
            "right_hip_now",
            "right_hip_target",
            "l_foot_x",
            "cmd_vel_x",
            "base_q_w",
        ):
            self.assertIn(payload_name, debug_source)
        self.assertIn("LW_SIM_DEBUG_FIELD_COUNT = 43", debug_header)
        self.assertIn("InitializePlotDebugResourcesLocked();", constructor)

    def test_pd_ff_mujoco_torques_are_validated_transactionally(self) -> None:
        source = SIM_SOURCE.read_text(encoding="utf-8")
        adapter = MUJOCO_ADAPTER_SOURCE.read_text(encoding="utf-8")
        command = source[
            source.index("void RL_Real::SetCommand(") : source.index(
                "void RL_Real::SetupSysJoystick("
            )
        ]

        self.assertIn("command->motor_command.tau", command)
        self.assertIn("command->motor_command.kp", command)
        self.assertIn("command->motor_command.kd", command)
        self.assertNotIn("actuator_net", command)
        self.assertIn("mujoco_control_adapter_->ApplyCommand(", command)
        self.assertNotIn("sensordata[", command)
        self.assertNotIn("mj_data->ctrl[", command)

        prepare = adapter.index("PrepareLWSimTorques(")
        first_write = adapter.index("data.ctrl[")
        self.assertLess(prepare, first_write)
        self.assertIn("if (!validation.valid())", adapter[prepare:first_write])
        self.assertIn(
            "LWSafetyEvent::SimulationActuatorCommandInvalid",
            command,
        )
        self.assertNotIn("data.ctrl[", adapter[:prepare])

    def test_mujoco_hot_paths_use_validated_runtime_configuration(self) -> None:
        source = SIM_SOURCE.read_text(encoding="utf-8")
        state = source[
            source.index("void RL_Real::GetState(") : source.index(
                "void RL_Real::SetCommand("
            )
        ]
        command = source[
            source.index("void RL_Real::SetCommand(") : source.index(
                "void RL_Real::SetupSysJoystick("
            )
        ]
        joystick = source[
            source.index("void RL_Real::GetSysJoystick()") : source.index(
                "int main("
            )
        ]

        for hot_path in (state, command, joystick):
            self.assertNotIn("params.Get", hot_path)
        self.assertIn("mujoco_control_adapter_->ReadState(", state)
        self.assertNotIn("sensordata[", state)
        self.assertIn("GetLWBaseRuntimeConfiguration()", command)
        self.assertIn("GetLWBaseRuntimeConfiguration()", joystick)
        self.assertIn("runtime_configuration.wheel_mask", command)
        self.assertIn("runtime_configuration.torque_limits", command)
        self.assertNotIn("mj_data->ctrl[", command)
        self.assertIn("GetLWBaseRuntimeConfiguration().vel_command", joystick)


if __name__ == "__main__":
    unittest.main()
