#include "lw_terminal_keyboard.hpp"
#include "rl_sdk.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <termios.h>
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
    explicit FileDescriptor(int descriptor = -1) noexcept
        : descriptor_(descriptor)
    {
    }

    ~FileDescriptor()
    {
        if (descriptor_ >= 0)
        {
            ::close(descriptor_);
        }
    }

    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor& operator=(const FileDescriptor&) = delete;

    int get() const noexcept
    {
        return descriptor_;
    }

private:
    int descriptor_ = -1;
};

class KeyboardTestRL final : public RL
{
public:
    std::vector<float> Forward() override
    {
        return {};
    }

    void GetState(RobotState<float>*) override
    {
    }

    void SetCommand(const RobotCommand<float>*) override
    {
    }
};

void writeAll(int descriptor, const char* bytes, size_t size)
{
    size_t offset = 0;
    while (offset < size)
    {
        const ssize_t written = ::write(descriptor, bytes + offset, size - offset);
        require(written > 0, "terminal test write failed");
        offset += static_cast<size_t>(written);
    }
}

void testTerminalConfigurationInputAndRestoration()
{
    FileDescriptor master(::posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC));
    require(master.get() >= 0, "failed to open pseudo-terminal master");
    require(::grantpt(master.get()) == 0, "failed to grant pseudo-terminal");
    require(::unlockpt(master.get()) == 0, "failed to unlock pseudo-terminal");
    const char* slave_name = ::ptsname(master.get());
    require(slave_name != nullptr, "failed to resolve pseudo-terminal slave");

    FileDescriptor probe(::open(slave_name, O_RDWR | O_NOCTTY | O_CLOEXEC));
    require(probe.get() >= 0, "failed to open pseudo-terminal slave");

    termios expected{};
    require(::tcgetattr(probe.get(), &expected) == 0, "failed to read initial termios");
    expected.c_lflag |= ICANON | ECHO | ISIG;
    expected.c_cc[VMIN] = 1;
    expected.c_cc[VTIME] = 0;
    require(
        ::tcsetattr(probe.get(), TCSANOW, &expected) == 0,
        "failed to establish initial termios");

    KeyboardTestRL rl;
    {
        LWTerminalKeyboard keyboard(slave_name);
        const int descriptor_flags = ::fcntl(keyboard.descriptor(), F_GETFL);
        require(descriptor_flags >= 0, "failed to inspect keyboard descriptor");
        require(
            (descriptor_flags & O_NONBLOCK) != 0,
            "terminal keyboard descriptor is blocking");

        termios configured{};
        require(
            ::tcgetattr(keyboard.descriptor(), &configured) == 0,
            "failed to read configured termios");
        require((configured.c_lflag & ICANON) == 0, "canonical mode remained enabled");
        require((configured.c_lflag & ECHO) == 0, "terminal echo remained enabled");
        require((configured.c_lflag & ISIG) != 0, "terminal signals were disabled");
        require(configured.c_cc[VMIN] == 0, "terminal VMIN is not non-blocking");
        require(configured.c_cc[VTIME] == 0, "terminal VTIME is not non-blocking");

        const char get_down = '9';
        writeAll(master.get(), &get_down, 1);
        rl.KeyboardInterface(keyboard.descriptor(), false);
        require(
            rl.control.current_keyboard == Input::Keyboard::Num9,
            "terminal key 9 did not reach the keyboard control state");

        rl.control.ClearInput();
        const char arrow_up[] = {'\x1b', '[', 'A'};
        writeAll(master.get(), arrow_up, sizeof(arrow_up));
        rl.KeyboardInterface(keyboard.descriptor(), false);
        require(
            rl.control.current_keyboard == Input::Keyboard::Up,
            "terminal arrow sequence was not decoded");
    }

    termios restored{};
    require(::tcgetattr(probe.get(), &restored) == 0, "failed to read restored termios");
    require(restored.c_lflag == expected.c_lflag, "terminal local flags were not restored");
    require(restored.c_cc[VMIN] == expected.c_cc[VMIN], "terminal VMIN was not restored");
    require(restored.c_cc[VTIME] == expected.c_cc[VTIME], "terminal VTIME was not restored");
}

void testMissingTerminalIsRejected()
{
    bool rejected = false;
    try
    {
        LWTerminalKeyboard keyboard("/definitely/missing/lw-terminal");
    }
    catch (const std::runtime_error&)
    {
        rejected = true;
    }
    require(rejected, "missing terminal keyboard was accepted");
}
} // namespace

int main()
{
    try
    {
        testTerminalConfigurationInputAndRestoration();
        testMissingTerminalIsRejected();
        std::cout << "LW terminal keyboard tests passed" << std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW terminal keyboard tests failed: "
                  << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
