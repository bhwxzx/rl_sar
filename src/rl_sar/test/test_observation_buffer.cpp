/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "observation_buffer.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void Require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void RequireEqual(
    const std::vector<float>& actual,
    const std::vector<float>& expected,
    const std::string& message)
{
    Require(actual == expected, message);
}

template<typename Exception, typename Operation>
void RequireThrows(Operation&& operation, const std::string& message)
{
    try
    {
        operation();
    }
    catch (const Exception&)
    {
        return;
    }
    catch (const std::exception& exception)
    {
        throw std::runtime_error(
            message + ": unexpected exception: " + exception.what());
    }
    throw std::runtime_error(message + ": exception was not thrown");
}

void TestTimePriorityPreservesRequestedFrameOrder()
{
    ObservationBuffer buffer(1, {2, 1}, 3, "time");
    buffer.insert({1.0F, 2.0F, 3.0F});
    buffer.insert({4.0F, 5.0F, 6.0F});
    buffer.insert({7.0F, 8.0F, 9.0F});

    RequireEqual(
        buffer.get_obs_vec({0, 2}),
        {7.0F, 8.0F, 9.0F, 1.0F, 2.0F, 3.0F},
        "time priority changed the requested history-frame order");
}

void TestTermPriorityPreservesRequestedFrameOrder()
{
    ObservationBuffer buffer(1, {2, 1}, 3, "term");
    buffer.insert({1.0F, 2.0F, 3.0F});
    buffer.insert({4.0F, 5.0F, 6.0F});
    buffer.insert({7.0F, 8.0F, 9.0F});

    RequireEqual(
        buffer.get_obs_vec({0, 2}),
        {7.0F, 8.0F, 1.0F, 2.0F, 9.0F, 3.0F},
        "term priority changed the observation-term or frame order");
}

void TestHistoryFrameIndexIsIndependentOfTermCount()
{
    ObservationBuffer buffer(1, {2}, 2, "time");
    buffer.insert({1.0F, 2.0F});
    buffer.insert({3.0F, 4.0F});

    RequireEqual(
        buffer.get_obs_vec({1}),
        {1.0F, 2.0F},
        "valid frame one was confused with observation term one");
}

void TestSparseHistoryAndMultipleEnvironmentsHaveExactSize()
{
    ObservationBuffer sparse(1, {1}, 4, "time");
    sparse.insert({10.0F});
    sparse.insert({20.0F});
    sparse.insert({30.0F});
    sparse.insert({40.0F});
    RequireEqual(
        sparse.get_obs_vec({3, 1}),
        {10.0F, 30.0F},
        "sparse history indices produced the wrong frames");

    ObservationBuffer multiple_environments(2, {1}, 2, "time");
    multiple_environments.insert({5.0F});
    multiple_environments.insert({6.0F});
    const auto output = multiple_environments.get_obs_vec({0, 1});
    RequireEqual(
        output,
        {6.0F, 5.0F, 6.0F, 5.0F},
        "multiple environments produced the wrong output order");
    Require(output.size() == 4, "multiple environments produced wrong size");
}

void TestEmptyAndInvalidHistoryRequests()
{
    ObservationBuffer buffer(1, {2}, 2, "time");
    Require(buffer.get_obs_vec({}).empty(), "empty history request was not empty");
    RequireThrows<std::out_of_range>(
        [&]() { static_cast<void>(buffer.get_obs_vec({-1})); },
        "negative history index was not rejected");
    RequireThrows<std::out_of_range>(
        [&]() { static_cast<void>(buffer.get_obs_vec({2})); },
        "history index at the buffer length was not rejected");
}

void TestObservationDimensionOverflowIsRejectedBeforeAllocation()
{
    RequireThrows<std::overflow_error>(
        []()
        {
            ObservationBuffer buffer(
                1,
                {std::numeric_limits<int>::max(), 1},
                1,
                "time");
            static_cast<void>(buffer);
        },
        "overflowing total observation dimension was not rejected");
}
} // namespace

int main()
{
    try
    {
        TestTimePriorityPreservesRequestedFrameOrder();
        TestTermPriorityPreservesRequestedFrameOrder();
        TestHistoryFrameIndexIsIndependentOfTermCount();
        TestSparseHistoryAndMultipleEnvironmentsHaveExactSize();
        TestEmptyAndInvalidHistoryRequests();
        TestObservationDimensionOverflowIsRejectedBeforeAllocation();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "test_observation_buffer failed: " << exception.what()
                  << std::endl;
        return 1;
    }
    std::cout << "test_observation_buffer passed" << std::endl;
    return 0;
}
