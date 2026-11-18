#include "lw_gamepad_mapping.hpp"

#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include <unistd.h>

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

class FileDescriptor
{
public:
    explicit FileDescriptor(int fd = -1) noexcept
        : fd_(fd)
    {
    }

    ~FileDescriptor()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
        }
    }

    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor& operator=(const FileDescriptor&) = delete;

    int get() const noexcept
    {
        return fd_;
    }

    void close() noexcept
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

private:
    int fd_;
};

struct Pipe
{
    FileDescriptor read_end;
    FileDescriptor write_end;
};

Pipe createPipe()
{
    int descriptors[2] = {-1, -1};
    require(::pipe(descriptors) == 0, "pipe creation failed");
    return {
        FileDescriptor(descriptors[0]),
        FileDescriptor(descriptors[1])
    };
}

std::string descriptorPath(int fd)
{
    return "/proc/self/fd/" + std::to_string(fd);
}

void writeAll(int fd, const void* data, size_t size)
{
    const auto* bytes = static_cast<const unsigned char*>(data);
    size_t offset = 0;
    while (offset < size)
    {
        const ssize_t written = ::write(fd, bytes + offset, size - offset);
        require(written > 0, "pipe write failed");
        offset += static_cast<size_t>(written);
    }
}

void testReadStatusDistinguishesIdleEventAndDisconnect()
{
    Pipe pipe = createPipe();
    LWJoystickDevice joystick(descriptorPath(pipe.read_end.get()));
    require(joystick.isFound(), "joystick did not open pipe descriptor");

    JoystickEvent event{};
    auto result = joystick.sample(&event);
    require(
        result.status == LWJoystickSampleStatus::NoData,
        "idle nonblocking descriptor was treated as disconnected");

    JoystickEvent sent{};
    sent.time = 123;
    sent.value = 1;
    sent.type = JS_EVENT_BUTTON;
    sent.number = 4;
    writeAll(pipe.write_end.get(), &sent, sizeof(sent));

    result = joystick.sample(&event);
    require(result.hasEvent(), "complete joystick event was not returned");
    require(
        event.time == sent.time
            && event.value == sent.value
            && event.type == sent.type
            && event.number == sent.number,
        "joystick event contents changed");

    pipe.write_end.close();
    result = joystick.sample(&event);
    require(
        result.status == LWJoystickSampleStatus::Disconnected,
        "pipe EOF was not treated as disconnect");
}

void testReadStatusRejectsInvalidAndPartialReads()
{
    LWJoystickDevice missing("/definitely/missing/lw-joystick");
    require(!missing.isFound(), "missing joystick path unexpectedly opened");

    JoystickEvent event{};
    auto result = missing.sample(&event);
    require(
        result.status == LWJoystickSampleStatus::Disconnected
            && result.error_number == EBADF,
        "invalid descriptor was not reported as disconnected");

    result = missing.sample(nullptr);
    require(
        result.status == LWJoystickSampleStatus::Error
            && result.error_number == EINVAL,
        "null event destination was accepted");

    Pipe pipe = createPipe();
    LWJoystickDevice partial(descriptorPath(pipe.read_end.get()));
    const unsigned char byte = 0x5a;
    writeAll(pipe.write_end.get(), &byte, sizeof(byte));
    result = partial.sample(&event);
    require(
        result.status == LWJoystickSampleStatus::Error
            && result.error_number == EPROTO
            && result.bytes_read == 1,
        "partial joystick event was not rejected");
}

void testEventBoundsAndDeadzone()
{
    std::array<LWJoystickButton, LW_JOYSTICK_BUTTON_COUNT> buttons{};
    std::array<int, LW_JOYSTICK_AXIS_COUNT> axes{};
    constexpr int max_axis_value = 32768;
    constexpr float deadzone = 0.05f;

    JoystickEvent event{};
    event.type = JS_EVENT_BUTTON;
    event.number = 19;
    event.value = 1;
    auto result =
        LWApplyJoystickEvent(event, buttons, axes, max_axis_value, deadzone);
    require(result == LWJoystickEventResult::Applied, "last valid button was rejected");
    require(buttons[19].pressed && buttons[19].on_press, "button state was not applied");

    LWBeginJoystickCycle(buttons);
    require(buttons[19].pressed, "cycle reset cleared held button");
    require(!buttons[19].on_press && !buttons[19].on_release, "cycle reset kept transient state");

    event.number = 20;
    result = LWApplyJoystickEvent(event, buttons, axes, max_axis_value, deadzone);
    require(
        result == LWJoystickEventResult::ButtonOutOfRange,
        "out-of-range button was accepted");

    event.type = JS_EVENT_AXIS;
    event.number = 9;
    event.value = 1638;
    result = LWApplyJoystickEvent(event, buttons, axes, max_axis_value, deadzone);
    require(result == LWJoystickEventResult::Applied, "last valid axis was rejected");
    require(axes[9] == 0, "existing five-percent deadzone was not preserved");

    event.value = 1639;
    result = LWApplyJoystickEvent(event, buttons, axes, max_axis_value, deadzone);
    require(result == LWJoystickEventResult::Applied, "axis outside deadzone was rejected");
    require(axes[9] == 1639, "axis outside deadzone was cleared");

    event.number = 10;
    result = LWApplyJoystickEvent(event, buttons, axes, max_axis_value, deadzone);
    require(
        result == LWJoystickEventResult::AxisOutOfRange,
        "out-of-range axis was accepted");

    event.type = 0;
    event.number = 0;
    result = LWApplyJoystickEvent(event, buttons, axes, max_axis_value, deadzone);
    require(result == LWJoystickEventResult::Unsupported, "unknown event type was applied");

    LWClearJoystickState(buttons, axes);
    for (const auto& button : buttons)
    {
        require(
            !button.pressed && !button.on_press && !button.on_release,
            "joystick clear kept a button state");
    }
    for (int axis : axes)
    {
        require(axis == 0, "joystick clear kept an axis value");
    }
}

void testGamepadEdges(const LWGamepadLayout& layout)
{
    using Key = Input::Gamepad;
    std::array<LWJoystickButton, LW_JOYSTICK_BUTTON_COUNT> buttons{};
    std::array<int, LW_JOYSTICK_AXIS_COUNT> axes{};
    const auto send = [&](unsigned char type, unsigned char id, short value) {
        Key key;
        require(LWReadGamepadEvent(layout, {0, value, type, id}, buttons, axes,
                                   32768, 0.05f, key) == LWJoystickEventResult::Applied,
                "valid input was rejected");
        return key;
    };
    const auto press = [&](unsigned char id) { return send(JS_EVENT_BUTTON, id, 1); };
    const auto release = [&](unsigned char id) {
        require(send(JS_EVENT_BUTTON, id, 0) == Key::None, "release generated an action");
    };
    require(send(JS_EVENT_AXIS, 1, -16384) == Key::None, "stick generated a button");
    for (bool to_wheel : {false, true})
    {
        const auto face = to_wheel ? layout.y : layout.a;
        const auto start = to_wheel ? Key::RB_Y : Key::RB_A;
        const auto reset = to_wheel ? Key::LB_Y : Key::LB_A;
        const auto keyboard = to_wheel ? Input::Keyboard::R : Input::Keyboard::T;
        const std::string keyframe = to_wheel ? "home_leg" : "home_wheel";

        require(send(JS_EVENT_INIT | JS_EVENT_BUTTON, face, 1) == Key::None,
                "initial held face button generated an action");
        release(face);
        require(press(face) == (to_wheel ? Key::Y : Key::A), "plain face binding changed");
        release(face);
        press(layout.lb);
        require(press(face) == reset, "reset chord did not resolve exclusively");
        require(LWSimResetKeyframe(Input::Keyboard::None, reset) == keyframe
                    && LWSimResetKeyframe(keyboard, Key::None) == keyframe,
                "reset did not select its keyframe");
        release(face);
        release(layout.lb);
        press(layout.rb);
        const short dpad = to_wheel ? -32767 : 32767;
        require(send(JS_EVENT_AXIS, 7, dpad) == (to_wheel ? Key::RB_DPadUp : Key::RB_DPadDown),
                "reset followed by balance entry lost the direction");
        require(send(JS_EVENT_AXIS, 7, dpad) == Key::None, "held D-pad retriggered");
        send(JS_EVENT_AXIS, 7, 0);
        require(press(face) == start && !LWSimResetKeyframe(Input::Keyboard::None, start),
                "switch chord also requested a reset");
        require(press(face) == Key::None, "held face retriggered");
        release(layout.rb);
        require(press(face) == Key::None, "releasing the shoulder reinterpreted the face");
        release(face);
        press(layout.lb);
        press(layout.rb);
        require(press(face) == Key::None, "ambiguous chord accepted");
        release(layout.lb);
        release(layout.rb);
        release(face);
        require(axes[1] == -16384 && axes[layout.yaw_axis] == 0, "buttons changed stick axes");
    }
}

void testLateSwitchIsDiscarded()
{
    LWInputMailbox<Input::Gamepad> mailbox;
    LWPolicyActivation activation;
    activation.activated_at = std::chrono::steady_clock::now();
    for (auto key : {Input::Gamepad::RB_A, Input::Gamepad::RB_Y,
                     Input::Gamepad::RB_DPadLeft, Input::Gamepad::RB_DPadRight})
    {
        mailbox.publishEvent(key, activation.activated_at - std::chrono::milliseconds(1));
        mailbox.publishVelocity(0.5f, 0.0f, 0.0f);
        LWInputSnapshot<Input::Gamepad> input;
        require(mailbox.read(input) && LWPolicyGamepadEvent(input, &activation) == Input::Gamepad::None,
                "old switch was replayed after policy activation");
        mailbox.publishEvent(key, activation.activated_at + std::chrono::milliseconds(1));
        require(mailbox.read(input) && LWPolicyGamepadEvent(input, &activation) == key,
                "fresh switch was discarded");
    }
}

void testFaultLatchCannotRecoverAutomatically()
{
    LWJoystickFaultLatch latch;
    require(!latch.faulted(), "fault latch started faulted");
    require(latch.latch(), "first fault did not latch");
    require(latch.faulted(), "fault latch did not remain set");
    require(!latch.latch(), "second fault was treated as a new latch");
    require(latch.faulted(), "fault latch recovered without restart");
}
} // namespace

int main()
{
    try
    {
        testReadStatusDistinguishesIdleEventAndDisconnect();
        testReadStatusRejectsInvalidAndPartialReads();
        testEventBoundsAndDeadzone();
        testGamepadEdges(LW_GAMEPAD_1);
        testGamepadEdges(LW_GAMEPAD_2);
        testLateSwitchIsDiscarded();
        testFaultLatchCannotRecoverAutomatically();
        std::cout << "LW joystick safety tests passed" << std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW joystick safety tests failed: "
                  << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
