#!/usr/bin/env python3

from pathlib import Path
import re
import sys
import unittest


REAL_SOURCE = Path(sys.argv.pop(1)).resolve()
REAL_HEADER = Path(sys.argv.pop(1)).resolve()
LAUNCH_FILE = Path(sys.argv.pop(1)).resolve()
DEPLOYMENT_GUIDE = Path(sys.argv.pop(1)).resolve()
RUNTIME_CORE = Path(sys.argv.pop(1)).resolve()


class RealKeyboardIntegrationTests(unittest.TestCase):
    def test_keyboard_is_initialized_before_workers(self) -> None:
        source = REAL_SOURCE.read_text(encoding="utf-8")
        constructor = source[
            source.index("RL_Real::RL_Real(") : source.index("RL_Real::~RL_Real()")
        ]
        self.assertIn('declare_parameter<bool>(\n        "enable_keyboard"', constructor)
        self.assertIn("std::make_unique<LWTerminalKeyboard>()", constructor)
        self.assertLess(
            constructor.index("std::make_unique<LWTerminalKeyboard>()"),
            constructor.index("this->loop_control->start()"),
        )

    def test_control_thread_polls_keyboard_before_fsm(self) -> None:
        source = REAL_SOURCE.read_text(encoding="utf-8")
        control = source[
            source.index("void RL_Real::RobotControl()") : source.index(
                "void RL_Real::RunModel()"
            )
        ]
        self.assertIn("this->KeyboardInterface(", control)
        runtime_core = RUNTIME_CORE.read_text(encoding="utf-8")
        self.assertLess(
            runtime_core.index("call(hooks.apply_keyboard);"),
            runtime_core.index("rl_->StateController("),
        )
        lw_controller_calls = re.findall(
            r"rl_->StateController\(\s*&rl_->robot_state,"
            r"\s*&rl_->robot_command,\s*false\);",
            runtime_core,
        )
        self.assertEqual(len(lw_controller_calls), 2)
        self.assertNotIn("loop_keyboard", source)
        self.assertNotIn("loop_keyboard", REAL_HEADER.read_text(encoding="utf-8"))

    def test_joystick_fault_gate_keeps_keyboard_state(self) -> None:
        source = REAL_SOURCE.read_text(encoding="utf-8")
        gate = source[
            source.index("void RL_Real::ApplyJoystickFaultGate()") : source.index(
                "void RL_Real::GetSysJoystick()"
            )
        ]
        self.assertIn("current_gamepad", gate)
        self.assertNotIn("current_keyboard", gate)

    def test_launch_enables_keyboard_by_default(self) -> None:
        launch = LAUNCH_FILE.read_text(encoding="utf-8")
        self.assertIn("enable_keyboard = LaunchConfiguration('enable_keyboard')", launch)
        self.assertIn("'enable_keyboard': enable_keyboard", launch)
        declaration = launch[launch.index("'enable_keyboard',") :]
        self.assertIn("default_value='true'", declaration)

    def test_deployment_guide_documents_terminal_contract(self) -> None:
        guide = DEPLOYMENT_GUIDE.read_text(encoding="utf-8")
        self.assertIn("enable_keyboard:=false", guide)
        self.assertIn("/dev/tty", guide)
        self.assertIn("数字键 `9`", guide)
        self.assertIn("键盘不提供速度控制", guide)
        self.assertIn("`Space` 不会停车", guide)


if __name__ == "__main__":
    unittest.main()
