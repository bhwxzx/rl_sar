#pragma once

#include "lw_joystick_safety.hpp"
#include "rl_sdk.hpp"

struct LWGamepadLayout
{
    unsigned char a, b, x, y, lb, rb, left_stick, right_stick, yaw_axis;
};

inline constexpr LWGamepadLayout LW_GAMEPAD_1{0, 1, 3, 4, 6, 7, 13, 14, 2};
inline constexpr LWGamepadLayout LW_GAMEPAD_2{0, 1, 2, 3, 4, 5, 9, 10, 3};

// Resolve the modifier at the physical press edge, before reading the next
// device event. Releases and initialization packets only update device state.
inline LWJoystickEventResult LWReadGamepadEvent(
    const LWGamepadLayout& layout, const JoystickEvent& event,
    std::array<LWJoystickButton, LW_JOYSTICK_BUTTON_COUNT>& buttons,
    std::array<int, LW_JOYSTICK_AXIS_COUNT>& axes, int max_value, float deadzone,
    Input::Gamepad& action)
{
    using Key = Input::Gamepad;
    action = Key::None;
    const int old_axis = event.number < axes.size() ? axes[event.number] : 0;
    const auto result = LWApplyJoystickEvent(event, buttons, axes, max_value, deadzone);
    if (result != LWJoystickEventResult::Applied || (event.type & JS_EVENT_INIT))
        return result;

    const bool lb = buttons[layout.lb].pressed;
    const bool rb = buttons[layout.rb].pressed;
    const auto chord = [&](Key plain, Key left, Key right) {
        if (rb) return right;
        return lb ? left : plain;
    };

    if (event.type == JS_EVENT_BUTTON && buttons[event.number].on_press)
    {
        const auto id = event.number;
        if (id == layout.a || id == layout.y)
        {
            if (lb && rb) return result;
            action = id == layout.a ? chord(Key::A, Key::LB_A, Key::RB_A)
                                    : chord(Key::Y, Key::LB_Y, Key::RB_Y);
        }
        else if (id == layout.b) action = chord(Key::B, Key::LB_B, Key::RB_B);
        else if (id == layout.x) action = chord(Key::X, Key::LB_X, Key::RB_X);
        else if (id == layout.left_stick) action = chord(Key::LStick, Key::LB_LStick, Key::RB_LStick);
        else if (id == layout.right_stick) action = chord(Key::RStick, Key::LB_RStick, Key::RB_RStick);
        else if (id == layout.lb) action = Key::LB;
        else if (id == layout.rb) action = lb ? Key::LB_RB : Key::RB;
    }
    else if (event.type == JS_EVENT_AXIS && (event.number == 6 || event.number == 7))
    {
        const int value = axes[event.number];
        if (value == 0 || (old_axis != 0 && (old_axis > 0) == (value > 0))) return result;
        if (event.number == 6)
            action = value < 0 ? chord(Key::DPadLeft, Key::LB_DPadLeft, Key::RB_DPadLeft)
                               : chord(Key::DPadRight, Key::LB_DPadRight, Key::RB_DPadRight);
        else
            action = value < 0 ? chord(Key::DPadUp, Key::LB_DPadUp, Key::RB_DPadUp)
                               : chord(Key::DPadDown, Key::LB_DPadDown, Key::RB_DPadDown);
    }
    return result;
}

// Use the last observed empty device queue as a lower bound on press time.
// A queued press straddling policy entry must not reverse the new policy.
inline Input::Gamepad LWPolicyGamepadEvent(
    const LWInputSnapshot<Input::Gamepad>& input, const LWPolicyActivation* activation)
{
    using Key = Input::Gamepad;
    switch (input.event)
    {
    case Key::RB_Y:
    case Key::RB_A:
    case Key::RB_DPadLeft:
    case Key::RB_DPadRight:
        if (!activation || input.earliest_event_time < activation->activated_at) return Key::None;
        break;
    default:
        break;
    }
    return input.event;
}

inline const char* LWSimResetKeyframe(Input::Keyboard keyboard, Input::Gamepad gamepad)
{
    if (keyboard == Input::Keyboard::T || gamepad == Input::Gamepad::LB_A) return "home_wheel";
    if (keyboard == Input::Keyboard::R || gamepad == Input::Gamepad::LB_Y) return "home_leg";
    return nullptr;
}
