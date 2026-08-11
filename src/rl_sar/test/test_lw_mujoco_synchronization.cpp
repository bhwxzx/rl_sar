#include <mujoco/mujoco.h>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void testConcurrentSimulationAccessUsesOneStepBoundary()
{
    char error[1024] = {};
    mjModel* model = mj_loadXML(LW_TEST_SCENE, nullptr, error, sizeof(error));
    require(model != nullptr, std::string("failed to load LW scene: ") + error);
    mjData* data = mj_makeData(model);
    require(data != nullptr, "failed to allocate MuJoCo data");

    std::recursive_mutex mutex;
    std::atomic<bool> failed{false};
    constexpr int iterations = 2000;
    const int home_leg_key = mj_name2id(model, mjOBJ_KEY, "home_leg");
    const int left_site =
        mj_name2id(model, mjOBJ_SITE, "left_foot_site");
    require(home_leg_key >= 0, "home_leg keyframe is missing");
    require(left_site >= 0, "left foot debug site is missing");

    const auto physics = [&]()
    {
        for (int iteration = 0; iteration < iterations; ++iteration)
        {
            const std::unique_lock<std::recursive_mutex> lock(mutex);
            mj_step(model, data);
            if (!std::isfinite(data->time))
            {
                failed.store(true);
                return;
            }
        }
    };
    const auto control = [&]()
    {
        for (int iteration = 0; iteration < iterations; ++iteration)
        {
            const std::unique_lock<std::recursive_mutex> lock(mutex);
            if (iteration != 0 && iteration % 250 == 0)
            {
                mj_resetDataKeyframe(model, data, home_leg_key);
                mj_forward(model, data);
            }
            for (int actuator = 0; actuator < model->nu; ++actuator)
            {
                data->ctrl[actuator] =
                    0.05 * std::sin(0.01 * static_cast<double>(iteration));
            }
        }
    };
    const auto state_snapshot = [&]()
    {
        for (int iteration = 0; iteration < iterations; ++iteration)
        {
            const std::unique_lock<std::recursive_mutex> lock(mutex);
            for (int sensor = 0; sensor < model->nsensordata; ++sensor)
            {
                if (!std::isfinite(data->sensordata[sensor]))
                {
                    failed.store(true);
                    return;
                }
            }
        }
    };
    const auto debug_snapshot = [&]()
    {
        for (int iteration = 0; iteration < iterations; ++iteration)
        {
            const std::unique_lock<std::recursive_mutex> lock(mutex);
            for (int axis = 0; axis < 3; ++axis)
            {
                if (!std::isfinite(data->site_xpos[3 * left_site + axis]))
                {
                    failed.store(true);
                    return;
                }
            }
        }
    };

    std::thread physics_thread(physics);
    std::thread control_thread(control);
    std::thread state_thread(state_snapshot);
    std::thread debug_thread(debug_snapshot);
    physics_thread.join();
    control_thread.join();
    state_thread.join();
    debug_thread.join();

    {
        const std::unique_lock<std::recursive_mutex> lock(mutex);
        for (int actuator = 0; actuator < model->nu; ++actuator)
        {
            data->ctrl[actuator] = 0.0;
        }
        for (int actuator = 0; actuator < model->nu; ++actuator)
        {
            require(data->ctrl[actuator] == 0.0,
                    "terminal actuator output was not zero");
        }
    }
    require(!failed.load(), "a synchronized MuJoCo snapshot was non-finite");

    mj_deleteData(data);
    mj_deleteModel(model);
}
} // namespace

int main()
{
    try
    {
        testConcurrentSimulationAccessUsesOneStepBoundary();
        std::cout << "LW MuJoCo synchronization stress test passed" << std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW MuJoCo synchronization stress test failed: "
                  << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
