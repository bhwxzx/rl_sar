#include "lw_mujoco_control_adapter.hpp"

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
std::atomic<bool> count_allocations{false};
std::atomic<std::size_t> allocation_count{0};
}

void* operator new(std::size_t size)
{
    if (count_allocations.load(std::memory_order_relaxed))
    {
        allocation_count.fetch_add(1, std::memory_order_relaxed);
    }
    if (void* memory = std::malloc(size == 0 ? 1 : size))
    {
        return memory;
    }
    throw std::bad_alloc();
}

void* operator new[](std::size_t size)
{
    return ::operator new(size);
}

void operator delete(void* memory) noexcept
{
    std::free(memory);
}

void operator delete[](void* memory) noexcept
{
    std::free(memory);
}

void operator delete(void* memory, std::size_t) noexcept
{
    std::free(memory);
}

void operator delete[](void* memory, std::size_t) noexcept
{
    std::free(memory);
}

namespace
{
struct ModelDeleter
{
    void operator()(mjModel* model) const noexcept
    {
        mj_deleteModel(model);
    }
};

struct DataDeleter
{
    void operator()(mjData* data) const noexcept
    {
        mj_deleteData(data);
    }
};

using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

void Require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

ModelPtr LoadXml(const std::string& xml)
{
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    const std::string name = "lw_adapter_test.xml";
    const int added = mj_addBufferVFS(
        &vfs,
        name.c_str(),
        xml.data(),
        static_cast<int>(xml.size()));
    Require(added == 0, "failed to add test XML to MuJoCo VFS");
    char error[2048]{};
    mjModel* model = mj_loadXML(name.c_str(), &vfs, error, sizeof(error));
    mj_deleteVFS(&vfs);
    if (!model)
    {
        throw std::runtime_error(
            std::string("failed to compile test XML: ") + error);
    }
    return ModelPtr(model);
}

ModelPtr LoadFile(const std::string& path)
{
    char error[2048]{};
    mjModel* model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    if (!model)
    {
        throw std::runtime_error(
            "failed to load " + path + ": " + error);
    }
    return ModelPtr(model);
}

std::string TestXml(
    bool include_alpha_torque = true,
    bool wrong_alpha_position_type = false,
    bool duplicate_alpha_position = false,
    bool include_alpha_actuator = true,
    bool duplicate_alpha_actuator = false)
{
    std::string alpha_position = wrong_alpha_position_type
        ? "<framepos name='alpha_pos' objtype='body' objname='alpha_body'/>"
        : "<jointpos name='alpha_pos' joint='alpha_joint'/>";
    std::string alpha_torque = include_alpha_torque
        ? "<jointactuatorfrc name='alpha_torque' joint='alpha_joint'/>"
        : "";
    std::string alpha_actuator = include_alpha_actuator
        ? "<motor name='alpha_joint' joint='alpha_joint'/>"
        : "";
    std::string duplicate_position = duplicate_alpha_position
        ? "<jointpos name='alpha_pos_duplicate' joint='alpha_joint'/>"
        : "";
    std::string duplicate_actuator = duplicate_alpha_actuator
        ? "<motor name='alpha_shadow' joint='alpha_joint'/>"
        : "";

    return std::string(
        "<mujoco model='lw_adapter_test'>"
        "<option gravity='0 0 0'/>"
        "<worldbody>"
        "<body name='base' pos='0 0 1'>"
        "<freejoint/><geom type='sphere' size='0.1' mass='1'/>"
        "<site name='imu'/>"
        "<body name='alpha_body' pos='0 0 0.2'>"
        "<joint name='alpha_joint' type='hinge' axis='1 0 0'/>"
        "<geom type='capsule' size='0.03 0.1' mass='0.1'/>"
        "</body>"
        "<body name='beta_body' pos='0 0 -0.2'>"
        "<joint name='beta_joint' type='hinge' axis='0 1 0'/>"
        "<geom type='capsule' size='0.03 0.1' mass='0.1'/>"
        "</body>"
        "<body name='extra_body' pos='0.2 0 0'>"
        "<joint name='extra_joint' type='hinge' axis='0 0 1'/>"
        "<geom type='capsule' size='0.03 0.1' mass='0.1'/>"
        "</body>"
        "</body>"
        "</worldbody>"
        "<actuator>"
        "<motor name='beta_joint' joint='beta_joint'/>"
        "<motor name='extra_joint' joint='extra_joint'/>"
        + alpha_actuator + duplicate_actuator
        + "</actuator>"
          "<sensor>"
          "<gyro name='imu_gyro' site='imu'/>"
          "<jointvel name='beta_vel' joint='beta_joint'/>"
          + alpha_position
          + "<jointactuatorfrc name='beta_torque' joint='beta_joint'/>"
            "<framequat name='imu_quat' objtype='site' objname='imu'/>"
            "<jointpos name='beta_pos' joint='beta_joint'/>"
            "<jointvel name='alpha_vel' joint='alpha_joint'/>"
          + alpha_torque + duplicate_position
        + "</sensor></mujoco>");
}

void SetSensor(mjModel& model, mjData& data, const char* name, double value)
{
    const int sensor_id = mj_name2id(&model, mjOBJ_SENSOR, name);
    Require(sensor_id >= 0, std::string("missing test sensor ") + name);
    data.sensordata[model.sensor_adr[sensor_id]] = value;
}

void ExpectRejected(const std::string& xml, const std::string& diagnostic)
{
    ModelPtr model = LoadXml(xml);
    bool rejected = false;
    try
    {
        LWMuJoCoControlAdapter adapter(
            *model,
            {"alpha_joint", "beta_joint"},
            {0, 1});
    }
    catch (const std::runtime_error& error)
    {
        rejected = std::string(error.what()).find(diagnostic)
            != std::string::npos;
    }
    Require(rejected, "invalid layout did not report: " + diagnostic);
}

void TestMaintainedScenes()
{
    const std::vector<std::string> joint_names = {
        "right_hip_joint", "left_hip_joint",
        "right_thigh_joint", "left_thigh_joint",
        "right_shank_joint", "left_shank_joint",
        "right_foot_joint", "left_foot_joint",
        "right_wheel_joint", "left_wheel_joint"};
    const std::vector<int> mapping = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    for (const char* scene : {"scene.xml", "scene_terrain.xml"})
    {
        ModelPtr model = LoadFile(
            std::string(LW_DESCRIPTION_MJCF_DIR) + "/" + scene);
        LWMuJoCoControlAdapter adapter(*model, joint_names, mapping);
        Require(adapter.numDofs() == joint_names.size(),
                std::string(scene) + " did not map every policy joint");
    }
}

void TestReorderedLayoutAndSafetyActions()
{
    ModelPtr model = LoadXml(TestXml());
    DataPtr data(mj_makeData(model.get()));
    Require(data != nullptr, "failed to allocate adapter test data");
    LWMuJoCoControlAdapter adapter(
        *model,
        {"alpha_joint", "beta_joint"},
        {1, 0});

    SetSensor(*model, *data, "beta_pos", 2.0);
    SetSensor(*model, *data, "beta_vel", 0.25);
    SetSensor(*model, *data, "beta_torque", 4.0);
    SetSensor(*model, *data, "alpha_pos", 1.0);
    SetSensor(*model, *data, "alpha_vel", -0.5);
    SetSensor(*model, *data, "alpha_torque", 3.0);
    const int quaternion_id = mj_name2id(model.get(), mjOBJ_SENSOR, "imu_quat");
    const int quaternion_address = model->sensor_adr[quaternion_id];
    data->sensordata[quaternion_address] = 1.0;
    const int gyro_id = mj_name2id(model.get(), mjOBJ_SENSOR, "imu_gyro");
    const int gyro_address = model->sensor_adr[gyro_id];
    data->sensordata[gyro_address + 2] = 0.75;

    std::vector<float> q(2);
    std::vector<float> dq(2);
    std::vector<float> tau(2);
    std::vector<float> quaternion(4);
    std::vector<float> gyroscope(3);
    Require(
        adapter.ReadState(*data, q, dq, tau, quaternion, gyroscope),
        "adapter rejected correctly sized state buffers");
    Require(q[0] == 2.0f && q[1] == 1.0f,
            "policy mapping did not reorder joint positions");
    Require(dq[0] == 0.25f && dq[1] == -0.5f,
            "policy mapping did not reorder joint velocities");
    Require(tau[0] == 4.0f && tau[1] == 3.0f,
            "policy mapping did not reorder joint torques");
    Require(quaternion[0] == 1.0f && gyroscope[2] == 0.75f,
            "named IMU sensors were not read");

    allocation_count.store(0, std::memory_order_relaxed);
    count_allocations.store(true, std::memory_order_relaxed);
    const bool allocation_free_read =
        adapter.ReadState(*data, q, dq, tau, quaternion, gyroscope);
    count_allocations.store(false, std::memory_order_relaxed);
    Require(allocation_free_read, "allocation check state read failed");
    Require(allocation_count.load(std::memory_order_relaxed) == 0,
            "adapter state path allocated memory");

    std::vector<float> desired_q = q;
    std::vector<float> desired_dq(2, 0.0f);
    std::vector<float> feedforward(2, 0.0f);
    std::vector<float> kp(2, 0.0f);
    std::vector<float> kd(2, 5.0f);
    std::vector<float> limits(2, 100.0f);
    std::vector<float> candidates(2);
    std::vector<float> bounded(2);
    const int beta_actuator =
        mj_name2id(model.get(), mjOBJ_ACTUATOR, "beta_joint");
    const int alpha_actuator =
        mj_name2id(model.get(), mjOBJ_ACTUATOR, "alpha_joint");
    const int extra_actuator =
        mj_name2id(model.get(), mjOBJ_ACTUATOR, "extra_joint");
    data->ctrl[extra_actuator] = 7.0;

    allocation_count.store(0, std::memory_order_relaxed);
    count_allocations.store(true, std::memory_order_relaxed);
    const LWSimTorqueValidation validation = adapter.ApplyCommand(
        *data,
        desired_q,
        desired_dq,
        feedforward,
        kp,
        kd,
        limits,
        candidates,
        bounded);
    count_allocations.store(false, std::memory_order_relaxed);
    Require(validation.valid(), "passive damping command was rejected");
    Require(allocation_count.load(std::memory_order_relaxed) == 0,
            "adapter command path allocated memory");
    Require(data->ctrl[beta_actuator] == -1.25,
            "S2 damping did not reach the mapped beta actuator");
    Require(data->ctrl[alpha_actuator] == 2.5,
            "S2 damping did not reach the mapped alpha actuator");
    Require(data->ctrl[extra_actuator] == 7.0,
            "normal command mutated an unrelated actuator");

    bool simulation_running = true;
    int simulation_run = 1;
    std::atomic<int> exit_request{0};
    std::fill(data->ctrl, data->ctrl + model->nu, 3.0);
    allocation_count.store(0, std::memory_order_relaxed);
    count_allocations.store(true, std::memory_order_relaxed);
    adapter.ExecuteSafetyDecision(
        *data,
        LWSafetyDecisionFor(LWSafetyEvent::MotorHardwareFault),
        {simulation_running, simulation_run, exit_request});
    count_allocations.store(false, std::memory_order_relaxed);
    for (int actuator = 0; actuator < model->nu; ++actuator)
    {
        Require(data->ctrl[actuator] == 0.0,
                "S3 did not zero every model actuator");
    }
    Require(simulation_running && simulation_run == 1
                && exit_request.load() == 0,
            "S3 incorrectly requested simulation shutdown");
    Require(allocation_count.load(std::memory_order_relaxed) == 0,
            "adapter safety path allocated memory");

    std::fill(data->ctrl, data->ctrl + model->nu, 4.0);
    adapter.ExecuteSafetyDecision(
        *data,
        LWSafetyDecisionFor(LWSafetyEvent::ControlLoopException),
        {simulation_running, simulation_run, exit_request});
    for (int actuator = 0; actuator < model->nu; ++actuator)
    {
        Require(data->ctrl[actuator] == 0.0,
                "S4 did not zero every model actuator");
    }
    Require(!simulation_running && simulation_run == 0
                && exit_request.load() == 1,
            "S4 shutdown did not propagate to every runtime target");
}

void TestInvalidLayoutsAreRejected()
{
    ExpectRejected(TestXml(false), "missing required sensor 'alpha_torque'");
    ExpectRejected(TestXml(true, true), "sensor 'alpha_pos' has the wrong type");
    ExpectRejected(TestXml(true, false, true), "sensor 'alpha_pos' is ambiguous");
    ExpectRejected(TestXml(true, false, false, false),
                   "missing required actuator 'alpha_joint'");
    ExpectRejected(TestXml(true, false, false, true, true),
                   "actuator 'alpha_joint' is ambiguous");

    {
        ModelPtr model = LoadXml(TestXml());
        const int sensor_id =
            mj_name2id(model.get(), mjOBJ_SENSOR, "alpha_pos");
        model->sensor_dim[sensor_id] = 2;
        bool rejected = false;
        try
        {
            LWMuJoCoControlAdapter adapter(
                *model,
                {"alpha_joint", "beta_joint"},
                {0, 1});
        }
        catch (const std::runtime_error& error)
        {
            rejected = std::string(error.what()).find("wrong dimension")
                != std::string::npos;
        }
        Require(rejected, "wrong sensor dimension was not rejected");
    }
    {
        ModelPtr model = LoadXml(TestXml());
        const int alpha_actuator =
            mj_name2id(model.get(), mjOBJ_ACTUATOR, "alpha_joint");
        const int beta_joint =
            mj_name2id(model.get(), mjOBJ_JOINT, "beta_joint");
        model->actuator_trnid[2 * alpha_actuator] = beta_joint;
        bool rejected = false;
        try
        {
            LWMuJoCoControlAdapter adapter(
                *model,
                {"alpha_joint", "beta_joint"},
                {0, 1});
        }
        catch (const std::runtime_error& error)
        {
            rejected = std::string(error.what()).find("wrong joint")
                != std::string::npos;
        }
        Require(rejected, "wrong actuator binding was not rejected");
    }
}
} // namespace

int main()
{
    try
    {
        TestMaintainedScenes();
        TestReorderedLayoutAndSafetyActions();
        TestInvalidLayoutsAreRejected();
    }
    catch (const std::exception& error)
    {
        std::cerr << "test_lw_mujoco_control_adapter failed: "
                  << error.what() << std::endl;
        return 1;
    }
    std::cout << "test_lw_mujoco_control_adapter passed" << std::endl;
    return 0;
}
