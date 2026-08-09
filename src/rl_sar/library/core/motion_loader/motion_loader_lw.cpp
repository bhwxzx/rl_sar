#include "motion_loader_lw.hpp"

#include "logger.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
std::string trim(const std::string& value)
{
    const auto first = std::find_if_not(
        value.begin(), value.end(), [](unsigned char character) {
            return std::isspace(character) != 0;
        });
    const auto last = std::find_if_not(
        value.rbegin(), value.rend(), [](unsigned char character) {
            return std::isspace(character) != 0;
        }).base();
    return first < last ? std::string(first, last) : std::string();
}

[[noreturn]] void csvError(
    const std::string& filename,
    std::size_t row,
    const std::string& message)
{
    throw std::runtime_error(
        "Invalid LW motion CSV '" + filename + "' at row "
        + std::to_string(row) + ": " + message);
}

float parseFiniteValue(
    const std::string& token,
    const std::string& filename,
    std::size_t row,
    std::size_t column)
{
    const std::string trimmed = trim(token);
    if (trimmed.empty())
    {
        csvError(
            filename,
            row,
            "column " + std::to_string(column) + " is empty");
    }

    std::size_t consumed = 0;
    float value = 0.0f;
    try
    {
        value = std::stof(trimmed, &consumed);
    }
    catch (const std::exception& exception)
    {
        csvError(
            filename,
            row,
            "column " + std::to_string(column)
                + " is not a valid float ('" + trimmed + "'): "
                + exception.what());
    }
    if (consumed != trimmed.size())
    {
        csvError(
            filename,
            row,
            "column " + std::to_string(column)
                + " contains trailing text: '" + trimmed + "'");
    }
    if (!std::isfinite(value))
    {
        csvError(
            filename,
            row,
            "column " + std::to_string(column) + " must be finite");
    }
    return value;
}
} // namespace

MotionLoaderLW::MotionLoaderLW(
    const std::string& motion_file,
    float fps,
    int time_offset_frames,
    std::size_t expected_num_joints)
    : num_frames_(0),
      num_joints_(expected_num_joints),
      time_offset_frames_(time_offset_frames),
      dt_(0.0f),
      duration_(0.0f),
      index_0_(0),
      index_1_(0),
      blend_(0.0f),
      world_to_init_{1.0f, 0.0f, 0.0f, 0.0f}
{
    if (!std::isfinite(fps) || fps <= 0.0f)
    {
        throw std::invalid_argument(
            "LW motion fps must be finite and positive");
    }
    if (time_offset_frames < 0)
    {
        throw std::invalid_argument(
            "LW motion time_offset_frames must be nonnegative");
    }
    if (expected_num_joints == 0)
    {
        throw std::invalid_argument(
            "LW motion expected_num_joints must be positive");
    }

    dt_ = 1.0f / fps;
    LoadFromCSV(motion_file);
    ComputeVelocities();

    num_frames_ = root_positions_.size();
    const double last_source_frame =
        static_cast<double>(time_offset_frames_)
        + static_cast<double>(num_frames_ - 1);
    const double duration = last_source_frame * static_cast<double>(dt_);
    if (!std::isfinite(duration)
        || duration > static_cast<double>(std::numeric_limits<float>::max()))
    {
        throw std::overflow_error("LW motion duration overflows float");
    }
    duration_ = static_cast<float>(duration);
    if (!std::isfinite(duration_) || duration_ <= 0.0f)
    {
        throw std::runtime_error(
            "LW motion duration must be finite and positive");
    }

    std::cout << LOGGER::INFO << "MotionLoaderLW: Loaded " << num_frames_
              << " frames, " << num_joints_
              << " joints, first-frame-time="
              << static_cast<float>(time_offset_frames_) * dt_
              << "s, duration=" << duration_ << "s" << std::endl;
}

void MotionLoaderLW::LoadFromCSV(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open motion file: " + filename);
    }

    const std::size_t expected_columns = 7 + num_joints_;
    std::vector<std::vector<float>> rows;
    std::string line;
    std::size_t row_number = 0;
    while (std::getline(file, line))
    {
        ++row_number;
        if (trim(line).empty())
        {
            csvError(filename, row_number, "row is empty");
        }

        std::stringstream stream(line);
        std::string token;
        std::vector<float> row;
        std::size_t column = 0;
        while (std::getline(stream, token, ','))
        {
            ++column;
            row.push_back(
                parseFiniteValue(
                    token, filename, row_number, column));
        }
        if (!line.empty() && line.back() == ',')
        {
            csvError(
                filename,
                row_number,
                "column " + std::to_string(column + 1) + " is empty");
        }
        if (row.size() != expected_columns)
        {
            csvError(
                filename,
                row_number,
                "expected " + std::to_string(expected_columns)
                    + " columns (7 root + "
                    + std::to_string(num_joints_) + " joints), got "
                    + std::to_string(row.size()));
        }

        const float quaternion_norm_squared =
            row[3] * row[3] + row[4] * row[4]
            + row[5] * row[5] + row[6] * row[6];
        if (!std::isfinite(quaternion_norm_squared)
            || quaternion_norm_squared <= 1.0e-12f)
        {
            csvError(
                filename,
                row_number,
                "root quaternion norm is too small");
        }
        rows.push_back(std::move(row));
    }

    if (rows.empty())
    {
        throw std::runtime_error(
            "LW motion CSV contains no frames: " + filename);
    }
    if (rows.size() < 2)
    {
        throw std::runtime_error(
            "LW motion CSV requires at least two frames: " + filename);
    }

    root_positions_.reserve(rows.size());
    root_quaternions_.reserve(rows.size());
    joint_positions_.reserve(rows.size());
    for (const auto& row : rows)
    {
        root_positions_.push_back({row[0], row[1], row[2]});
        root_quaternions_.push_back(
            QuaternionNormalize({row[6], row[3], row[4], row[5]}));
        joint_positions_.emplace_back(row.begin() + 7, row.end());
    }
}

void MotionLoaderLW::ComputeVelocities()
{
    joint_velocities_.assign(
        joint_positions_.size(),
        std::vector<float>(num_joints_, 0.0f));
    for (std::size_t frame = 0;
         frame + 1 < joint_positions_.size();
         ++frame)
    {
        for (std::size_t joint = 0; joint < num_joints_; ++joint)
        {
            joint_velocities_[frame][joint] =
                (joint_positions_[frame + 1][joint]
                 - joint_positions_[frame][joint])
                / dt_;
        }
    }
    joint_velocities_.back() =
        joint_velocities_[joint_velocities_.size() - 2];
}

void MotionLoaderLW::Update(float time)
{
    if (!std::isfinite(time))
    {
        throw std::invalid_argument("LW motion update time must be finite");
    }
    const float clamped_time = std::clamp(time, 0.0f, duration_);
    const float last_frame = static_cast<float>(num_frames_ - 1);
    const float frame_float = std::clamp(
        clamped_time / dt_
            - static_cast<float>(time_offset_frames_),
        0.0f,
        last_frame);

    index_0_ = static_cast<std::size_t>(std::floor(frame_float));
    index_1_ = std::min(index_0_ + 1, num_frames_ - 1);
    blend_ = frame_float - static_cast<float>(index_0_);
}

void MotionLoaderLW::Reset(const std::vector<float>& robot_base_quat)
{
    Update(0.0f);

    const std::vector<float> robot_torso =
        ComputeTorsoQuat(robot_base_quat);
    const std::vector<float> motion_torso = GetAnchorQuat();
    world_to_init_ = ComputeYawAlignment(robot_torso, motion_torso);

    std::cout << LOGGER::INFO
              << "Motion reset with yaw alignment" << std::endl;
}

std::vector<float> MotionLoaderLW::GetJointPos() const
{
    std::vector<float> result;
    result.reserve(num_joints_);
    const auto& pos0 = joint_positions_[index_0_];
    const auto& pos1 = joint_positions_[index_1_];

    for (std::size_t joint = 0; joint < num_joints_; ++joint)
    {
        result.push_back(
            pos0[joint] * (1.0f - blend_)
            + pos1[joint] * blend_);
    }
    return result;
}

std::vector<float> MotionLoaderLW::GetJointVel() const
{
    std::vector<float> result;
    result.reserve(num_joints_);
    const auto& vel0 = joint_velocities_[index_0_];
    const auto& vel1 = joint_velocities_[index_1_];

    for (std::size_t joint = 0; joint < num_joints_; ++joint)
    {
        result.push_back(
            vel0[joint] * (1.0f - blend_)
            + vel1[joint] * blend_);
    }
    return result;
}

std::vector<float> MotionLoaderLW::GetRootQuat() const
{
    return Slerp(
        root_quaternions_[index_0_],
        root_quaternions_[index_1_],
        blend_);
}

std::vector<float> MotionLoaderLW::ComputeTorsoQuat(
    const std::vector<float>& base_quat)
{
    return QuaternionNormalize(base_quat);
}

std::vector<float> MotionLoaderLW::ComputeYawAlignment(
    const std::vector<float>& robot_torso_quat,
    const std::vector<float>& motion_torso_quat)
{
    const std::vector<float> robot_yaw =
        QuaternionYawOnly(robot_torso_quat);
    const std::vector<float> motion_yaw =
        QuaternionYawOnly(motion_torso_quat);
    return QuaternionMultiply(
        robot_yaw, QuaternionConjugate(motion_yaw));
}

std::vector<float> MotionLoaderLW::GetAnchorQuat() const
{
    return ComputeTorsoQuat(GetRootQuat());
}

std::vector<float> MotionLoaderLW::Slerp(
    const std::vector<float>& q0,
    const std::vector<float>& q1,
    float t) const
{
    float dot = q0[0] * q1[0] + q0[1] * q1[1]
        + q0[2] * q1[2] + q0[3] * q1[3];

    std::vector<float> q1_adjusted = q1;
    if (dot < 0.0f)
    {
        q1_adjusted = {-q1[0], -q1[1], -q1[2], -q1[3]};
        dot = -dot;
    }

    if (dot > 0.9995f)
    {
        return QuaternionNormalize({
            q0[0] + t * (q1_adjusted[0] - q0[0]),
            q0[1] + t * (q1_adjusted[1] - q0[1]),
            q0[2] + t * (q1_adjusted[2] - q0[2]),
            q0[3] + t * (q1_adjusted[3] - q0[3])});
    }

    const float theta = std::acos(std::clamp(dot, -1.0f, 1.0f));
    const float sin_theta = std::sin(theta);
    const float weight_0 = std::sin((1.0f - t) * theta) / sin_theta;
    const float weight_1 = std::sin(t * theta) / sin_theta;

    return QuaternionNormalize({
        q0[0] * weight_0 + q1_adjusted[0] * weight_1,
        q0[1] * weight_0 + q1_adjusted[1] * weight_1,
        q0[2] * weight_0 + q1_adjusted[2] * weight_1,
        q0[3] * weight_0 + q1_adjusted[3] * weight_1});
}
