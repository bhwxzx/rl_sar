#include "fsm_LW.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr float kTolerance = 1.0e-5f;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void requireNear(
    float actual,
    float expected,
    const std::string& message)
{
    if (std::fabs(actual - expected) > kTolerance)
    {
        std::ostringstream details;
        details << message
                << ": expected " << expected
                << ", got " << actual;
        throw std::runtime_error(details.str());
    }
}

std::vector<std::vector<float>> loadCSV(
    const std::string& path)
{
    std::ifstream file(path);
    require(file.is_open(), "failed to open motion CSV " + path);

    std::vector<std::vector<float>> rows;
    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream stream(line);
        std::string value;
        std::vector<float> row;
        while (std::getline(stream, value, ','))
        {
            row.push_back(std::stof(value));
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

void verifyMotion(
    const std::string& policy_name,
    size_t expected_frames)
{
    const std::string policy_path =
        std::string(POLICY_DIR)
        + "/LW/robot_lab/" + policy_name;
    const std::string config_path =
        policy_path + "/config.yaml";
    const std::string config_key =
        "LW/robot_lab/" + policy_name;
    const YAML::Node config =
        YAML::LoadFile(config_path)[config_key];
    require(
        config.IsDefined(),
        "missing policy config " + config_key);

    YamlParams policy_params;
    policy_params.config_node = config;
    const float source_fps =
        LW_fsm::GetLWMotionSourceFPS(policy_params);
    const int time_offset_frames =
        policy_params.Get<int>("motion_time_offset_frames");
    requireNear(
        source_fps,
        60.0f,
        policy_name + " did not use the configured source rate");
    require(
        time_offset_frames == 1,
        policy_name + " did not record the removed t=0 source frame");

    const std::string motion_path =
        policy_path + "/"
        + config["motion_file"].as<std::string>();
    const auto rows = loadCSV(motion_path);
    require(
        rows.size() == expected_frames,
        policy_name + " has an unexpected frame count");
    require(
        rows.size() >= 2,
        policy_name + " needs at least two frames");
    const size_t columns = rows.front().size();
    require(
        columns > 7,
        policy_name + " has no joint columns");
    for (const auto& row : rows)
    {
        require(
            row.size() == columns,
            policy_name + " has inconsistent CSV row widths");
    }

    MotionLoaderLW loader(
        motion_path,
        source_fps,
        time_offset_frames,
        columns - 7);
    requireNear(
        loader.GetDuration(),
        static_cast<float>(
            time_offset_frames + rows.size() - 1)
            / source_fps,
        policy_name + " duration does not match its source rate");

    loader.Update(0.0f);
    const auto start_position = loader.GetJointPos();
    const auto start_velocity = loader.GetJointVel();
    require(
        start_position.size() == columns - 7
            && start_velocity.size() == columns - 7,
        policy_name + " produced an unexpected joint count");
    for (size_t joint = 0;
         joint < start_position.size();
         ++joint)
    {
        requireNear(
            start_position[joint],
            rows[0][joint + 7],
            policy_name + " start position is incorrect");
        requireNear(
            start_velocity[joint],
            (rows[1][joint + 7]
             - rows[0][joint + 7])
                * source_fps,
            policy_name + " velocity does not use 60 Hz");
    }

    loader.Update(
        static_cast<float>(time_offset_frames) / source_fps);
    require(
        loader.GetJointPos() == start_position,
        policy_name + " did not hold its first row before its source time");

    const size_t intermediate_index = rows.size() / 2;
    loader.Update(
        static_cast<float>(
            time_offset_frames + intermediate_index)
            / source_fps);
    const auto intermediate_position = loader.GetJointPos();
    for (size_t joint = 0;
         joint < intermediate_position.size();
         ++joint)
    {
        requireNear(
            intermediate_position[joint],
            rows[intermediate_index][joint + 7],
            policy_name + " source timestamp does not select its exact row");
    }

    loader.Update(
        (static_cast<float>(
             time_offset_frames + intermediate_index)
         + 0.5f)
        / source_fps);
    const auto midpoint_position = loader.GetJointPos();
    for (size_t joint = 0;
         joint < midpoint_position.size();
         ++joint)
    {
        const float expected =
            0.5f * rows[intermediate_index][joint + 7]
            + 0.5f * rows[intermediate_index + 1][joint + 7];
        requireNear(
            midpoint_position[joint],
            expected,
            policy_name + " midpoint interpolation is incorrect");
    }

    loader.Update(loader.GetDuration());
    const auto end_position = loader.GetJointPos();
    for (size_t joint = 0;
         joint < end_position.size();
         ++joint)
    {
        requireNear(
            end_position[joint],
            rows.back()[joint + 7],
            policy_name + " did not reach the final frame");
    }
}
} // namespace

int main()
{
    try
    {
        YamlParams distinct_rates;
        distinct_rates.config_node["dt"] = 0.005f;
        distinct_rates.config_node["decimation"] = 4;
        distinct_rates.config_node["motion_fps"] = 60.0f;
        requireNear(
            LW_fsm::GetLWMotionSourceFPS(distinct_rates),
            60.0f,
            "motion source rate was coupled to the 50 Hz policy rate");

        verifyMotion("leg_to_wheel", 167);
        verifyMotion("wheel_to_leg", 170);

        std::cout
            << "LW motion reference rate tests passed"
            << std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception)
    {
        std::cerr
            << "LW motion reference rate tests failed: "
            << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
