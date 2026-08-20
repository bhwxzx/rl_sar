#include "mujoco_utils.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
using namespace std::chrono_literals;

void Require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

class HeadlessPlatformUI final : public mujoco::PlatformUIAdapter
{
public:
    ~HeadlessPlatformUI() override
    {
        FreeMjrContext();
    }

    std::pair<double, double> GetCursorPosition() const override
    {
        return {0.0, 0.0};
    }
    double GetDisplayPixelsPerInch() const override { return 96.0; }
    std::pair<int, int> GetFramebufferSize() const override
    {
        return {640, 480};
    }
    std::pair<int, int> GetWindowSize() const override { return {640, 480}; }
    bool IsGPUAccelerated() const override { return false; }
    void PollEvents() override {}
    void SetClipboardString(const char*) override {}
    void SetVSync(bool) override {}
    void SetWindowTitle(const char*) override {}
    bool ShouldCloseWindow() const override { return true; }
    void SwapBuffers() override {}
    void ToggleFullscreen() override {}
    bool IsLeftMouseButtonPressed() const override { return false; }
    bool IsMiddleMouseButtonPressed() const override { return false; }
    bool IsRightMouseButtonPressed() const override { return false; }
    bool IsAltKeyPressed() const override { return false; }
    bool IsCtrlKeyPressed() const override { return false; }
    bool IsShiftKeyPressed() const override { return false; }
    bool IsMouseButtonDownEvent(int) const override { return false; }
    bool IsKeyDownEvent(int) const override { return false; }
    int TranslateKeyCode(int key) const override { return key; }
    mjtButton TranslateMouseButton(int) const override { return mjBUTTON_NONE; }
};

struct HeadlessSimulation
{
    HeadlessSimulation()
    {
        mjv_defaultCamera(&camera);
        mjv_defaultOption(&option);
        mjv_defaultPerturb(&perturb);
        simulate = std::make_unique<mujoco::Simulate>(
            std::make_unique<HeadlessPlatformUI>(),
            &camera,
            &option,
            &perturb,
            false);
    }

    mjvCamera camera;
    mjvOption option;
    mjvPerturb perturb;
    std::unique_ptr<mujoco::Simulate> simulate;
};

void TestInvalidSceneFailsPromptly()
{
    HeadlessSimulation headless;
    LWMuJoCoPhysicsLifecycle lifecycle(*headless.simulate);
    const std::string missing_scene =
        "/definitely-missing/lw024-invalid-scene.xml";
    const auto started = std::chrono::steady_clock::now();
    bool diagnosed = false;
    try
    {
        lifecycle.Start(missing_scene);
    }
    catch (const std::runtime_error& error)
    {
        const std::string diagnostic = error.what();
        diagnosed = diagnostic.find("Failed to load MuJoCo scene")
                != std::string::npos
            && diagnostic.find(missing_scene) != std::string::npos;
    }
    const auto elapsed = std::chrono::steady_clock::now() - started;
    Require(diagnosed, "invalid scene must preserve its MuJoCo diagnostic");
    Require(elapsed < 2s, "invalid scene startup must not poll indefinitely");
    Require(lifecycle.model() == nullptr, "invalid scene leaked a model");
    Require(lifecycle.data() == nullptr, "invalid scene leaked data");
}

void TestPendingRenderLoadCanBeCancelledAndJoined()
{
    HeadlessSimulation headless;
    LWMuJoCoPhysicsLifecycle lifecycle(*headless.simulate);
    lifecycle.Start(LW_TEST_SCENE);
    Require(lifecycle.model() != nullptr, "valid scene did not load a model");
    Require(lifecycle.data() != nullptr, "valid scene did not allocate data");

    const auto started = std::chrono::steady_clock::now();
    lifecycle.Stop();
    const auto elapsed = std::chrono::steady_clock::now() - started;
    Require(elapsed < 2s, "pending render load did not cancel promptly");
    Require(lifecycle.model() == nullptr, "shutdown retained a model");
    Require(lifecycle.data() == nullptr, "shutdown retained data");
    lifecycle.Stop();
}

void TestValidationFailurePreventsWorkerStartup()
{
    HeadlessSimulation headless;
    LWMuJoCoPhysicsLifecycle lifecycle(*headless.simulate);
    bool validator_called = false;
    bool diagnosed = false;
    try
    {
        lifecycle.Start(
            LW_TEST_SCENE,
            [&validator_called](const mjModel& model, mjData& data)
            {
                validator_called = model.nq >= 0 && data.time == 0.0;
                throw std::runtime_error("intentional layout rejection");
            });
    }
    catch (const std::runtime_error& error)
    {
        diagnosed = std::string(error.what()) == "intentional layout rejection";
    }
    Require(validator_called, "pre-start validator did not receive model data");
    Require(diagnosed, "pre-start validator diagnostic was not preserved");
    Require(lifecycle.model() == nullptr, "rejected model was published");
    Require(lifecycle.data() == nullptr, "rejected data was published");
    lifecycle.Stop();
}
} // namespace

int main()
{
    try
    {
        TestInvalidSceneFailsPromptly();
        TestValidationFailurePreventsWorkerStartup();
        TestPendingRenderLoadCanBeCancelledAndJoined();
    }
    catch (const std::exception& error)
    {
        std::cerr << "test_lw_mujoco_lifecycle failed: " << error.what()
                  << std::endl;
        return 1;
    }
    std::cout << "test_lw_mujoco_lifecycle passed" << std::endl;
    return 0;
}
