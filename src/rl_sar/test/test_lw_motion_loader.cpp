#include "motion_loader_lw.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

namespace
{
namespace fs = std::filesystem;

constexpr float kTolerance = 1.0e-5f;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void requireNear(float actual, float expected, const std::string& message)
{
    if (std::fabs(actual - expected) > kTolerance)
    {
        throw std::runtime_error(
            message + ": expected " + std::to_string(expected)
            + ", got " + std::to_string(actual));
    }
}

void requireVectorNear(
    const std::vector<float>& actual,
    const std::vector<float>& expected,
    const std::string& message)
{
    require(actual.size() == expected.size(), message + ": wrong size");
    for (std::size_t index = 0; index < actual.size(); ++index)
    {
        requireNear(
            actual[index],
            expected[index],
            message + " at index " + std::to_string(index));
    }
}

void requireFailure(
    const std::function<void()>& operation,
    const std::string& expected_text)
{
    try
    {
        operation();
    }
    catch (const std::exception& exception)
    {
        require(
            std::string(exception.what()).find(expected_text)
                != std::string::npos,
            "unexpected error: " + std::string(exception.what()));
        return;
    }
    throw std::runtime_error(
        "expected failure containing: " + expected_text);
}

class Fixture
{
public:
    Fixture()
    {
        root = fs::temp_directory_path()
            / ("lw-motion-loader-test-" + std::to_string(getpid()));
        fs::create_directories(root);
    }

    ~Fixture()
    {
        std::error_code error;
        fs::remove_all(root, error);
    }

    fs::path write(
        const std::string& name,
        const std::string& content) const
    {
        const fs::path path = root / name;
        std::ofstream output(path);
        if (!output)
        {
            throw std::runtime_error("failed to create " + path.string());
        }
        output << content;
        return path;
    }

    fs::path root;
};

const std::string kRowA = "0,0,0,0,0,0,1,1,2\n";
const std::string kRowB = "0,0,0,0,0,0,1,3,6\n";
const std::string kRowC = "0,0,0,0,0,0,1,5,10\n";
const std::string kRowD = "0,0,0,0,0,0,1,7,14\n";

void testOffsetOneTimeline(const Fixture& fixture)
{
    const fs::path file = fixture.write("offset-one.csv", kRowA + kRowB);
    MotionLoaderLW loader(file.string(), 10.0f, 1, 2);
    requireNear(loader.GetDuration(), 0.2f, "offset-one duration");

    loader.Update(-1.0f);
    requireVectorNear(loader.GetJointPos(), {1.0f, 2.0f}, "negative time");
    loader.Update(0.0f);
    requireVectorNear(loader.GetJointPos(), {1.0f, 2.0f}, "time zero");
    loader.Update(0.1f);
    requireVectorNear(loader.GetJointPos(), {1.0f, 2.0f}, "first source time");
    loader.Update(0.15f);
    requireVectorNear(loader.GetJointPos(), {2.0f, 4.0f}, "midpoint");
    requireVectorNear(loader.GetJointVel(), {20.0f, 40.0f}, "velocity");
    loader.Update(0.2f);
    requireVectorNear(loader.GetJointPos(), {3.0f, 6.0f}, "last source time");
    loader.Update(1.0f);
    requireVectorNear(loader.GetJointPos(), {3.0f, 6.0f}, "past duration");

    requireFailure(
        [&]() {
            loader.Update(std::numeric_limits<float>::quiet_NaN());
        },
        "update time must be finite");
}

void testOffsetZeroSupportsFutureCSV(const Fixture& fixture)
{
    const fs::path file = fixture.write("offset-zero.csv", kRowA + kRowB);
    MotionLoaderLW loader(file.string(), 10.0f, 0, 2);
    requireNear(loader.GetDuration(), 0.1f, "offset-zero duration");

    loader.Update(0.0f);
    requireVectorNear(loader.GetJointPos(), {1.0f, 2.0f}, "offset-zero start");
    loader.Update(0.05f);
    requireVectorNear(loader.GetJointPos(), {2.0f, 4.0f}, "offset-zero midpoint");
    loader.Update(0.1f);
    requireVectorNear(loader.GetJointPos(), {3.0f, 6.0f}, "offset-zero end");
}

void testManyFrameTimestamps(const Fixture& fixture)
{
    const fs::path file = fixture.write(
        "many.csv", kRowA + kRowB + kRowC + kRowD);
    MotionLoaderLW loader(file.string(), 10.0f, 1, 2);
    requireNear(loader.GetDuration(), 0.4f, "many-frame duration");

    const std::vector<std::vector<float>> expected = {
        {1.0f, 2.0f},
        {3.0f, 6.0f},
        {5.0f, 10.0f},
        {7.0f, 14.0f},
    };
    for (std::size_t row = 0; row < expected.size(); ++row)
    {
        loader.Update(static_cast<float>(row + 1) / 10.0f);
        requireVectorNear(
            loader.GetJointPos(),
            expected[row],
            "exact many-frame timestamp");
    }
}

void testConstructorAndCSVValidation(const Fixture& fixture)
{
    const fs::path valid = fixture.write("valid.csv", kRowA + kRowB);
    requireFailure(
        [&]() { MotionLoaderLW loader(valid.string(), 0.0f, 1, 2); },
        "fps must be finite and positive");
    requireFailure(
        [&]() {
            MotionLoaderLW loader(
                valid.string(),
                std::numeric_limits<float>::infinity(),
                1,
                2);
        },
        "fps must be finite and positive");
    requireFailure(
        [&]() { MotionLoaderLW loader(valid.string(), 10.0f, -1, 2); },
        "time_offset_frames must be nonnegative");
    requireFailure(
        [&]() { MotionLoaderLW loader(valid.string(), 10.0f, 1, 0); },
        "expected_num_joints must be positive");

    const fs::path empty = fixture.write("empty.csv", "");
    requireFailure(
        [&]() { MotionLoaderLW loader(empty.string(), 10.0f, 1, 2); },
        "contains no frames");
    const fs::path one = fixture.write("one.csv", kRowA);
    requireFailure(
        [&]() { MotionLoaderLW loader(one.string(), 10.0f, 1, 2); },
        "at least two frames");

    const fs::path bad_number = fixture.write(
        "bad-number.csv",
        "0,0,0,0,0,0,1,bad,2\n" + kRowB);
    requireFailure(
        [&]() {
            MotionLoaderLW loader(bad_number.string(), 10.0f, 1, 2);
        },
        "column 8 is not a valid float");
    const fs::path non_finite = fixture.write(
        "non-finite.csv",
        "0,0,0,0,0,0,1,nan,2\n" + kRowB);
    requireFailure(
        [&]() {
            MotionLoaderLW loader(non_finite.string(), 10.0f, 1, 2);
        },
        "column 8 must be finite");
    const fs::path wrong_width = fixture.write(
        "wrong-width.csv",
        kRowA + "0,0,0,0,0,0,1,3\n");
    requireFailure(
        [&]() {
            MotionLoaderLW loader(wrong_width.string(), 10.0f, 1, 2);
        },
        "expected 9 columns");
    const fs::path zero_quaternion = fixture.write(
        "zero-quaternion.csv",
        "0,0,0,0,0,0,0,1,2\n" + kRowB);
    requireFailure(
        [&]() {
            MotionLoaderLW loader(zero_quaternion.string(), 10.0f, 1, 2);
        },
        "quaternion norm is too small");
    const fs::path trailing_comma = fixture.write(
        "trailing-comma.csv",
        "0,0,0,0,0,0,1,1,2,\n" + kRowB);
    requireFailure(
        [&]() {
            MotionLoaderLW loader(trailing_comma.string(), 10.0f, 1, 2);
        },
        "column 10 is empty");
    const fs::path empty_row = fixture.write(
        "empty-row.csv", kRowA + "\n" + kRowB);
    requireFailure(
        [&]() {
            MotionLoaderLW loader(empty_row.string(), 10.0f, 1, 2);
        },
        "row is empty");
}
} // namespace

int main()
{
    try
    {
        const Fixture fixture;
        testOffsetOneTimeline(fixture);
        testOffsetZeroSupportsFutureCSV(fixture);
        testManyFrameTimestamps(fixture);
        testConstructorAndCSVValidation(fixture);
    }
    catch (const std::exception& exception)
    {
        std::cerr << "test_lw_motion_loader failed: "
                  << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "test_lw_motion_loader passed" << std::endl;
    return EXIT_SUCCESS;
}
