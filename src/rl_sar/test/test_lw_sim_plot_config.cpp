#include "lw_sim_plot_config.hpp"

#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
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

LWSimPlotConfiguration Parse(std::vector<std::string> arguments)
{
    std::vector<char*> argv;
    argv.reserve(arguments.size());
    for (std::string& argument : arguments)
    {
        argv.push_back(argument.data());
    }
    return ParseLWSimPlotConfiguration(
        static_cast<int>(argv.size()),
        argv.data());
}

void RequireFailure(
    std::vector<std::string> arguments,
    const std::string& expected_text)
{
    try
    {
        static_cast<void>(Parse(std::move(arguments)));
    }
    catch (const std::exception& exception)
    {
        Require(
            std::string(exception.what()).find(expected_text)
                != std::string::npos,
            "unexpected plot-configuration error: "
                + std::string(exception.what()));
        return;
    }
    throw std::runtime_error("expected plot-configuration failure");
}

void TestPlotDefaultsOffAtOneHundredHertz()
{
    const auto configuration = Parse({"rl_sim_LW"});
    Require(!configuration.enabled, "Sim2Sim plot defaulted on");
    Require(
        configuration.rate_hz == 100,
        "Sim2Sim plot default rate differs from 100 Hz");

    const auto unrelated = Parse(
        {"rl_sim_LW", "--policy-root", "/tmp/policy", "--use_actuator_net"});
    Require(!unrelated.enabled, "unrelated arguments enabled plotting");
}

void TestExplicitPlotRatesAndPeriods()
{
    const auto default_enabled = Parse({"rl_sim_LW", "--enable-plot"});
    Require(default_enabled.enabled, "--enable-plot was ignored");
    Require(
        LWSimPlotPeriod(default_enabled) == std::chrono::milliseconds(10),
        "100 Hz plot period differs from 10 ms");

    for (const int rate : {1, 50, 100, 200})
    {
        const auto configuration = Parse(
            {"rl_sim_LW",
             "--plot-rate-hz",
             std::to_string(rate),
             "--enable-plot"});
        Require(configuration.enabled, "valid plot rate did not enable plotting");
        Require(configuration.rate_hz == rate, "valid plot rate changed");
    }

    const auto repeated = Parse(
        {"rl_sim_LW",
         "--enable-plot",
         "--plot-rate-hz",
         "50",
         "--plot-rate-hz",
         "50"});
    Require(repeated.rate_hz == 50, "identical repeated plot rate changed");
}

void TestInvalidPlotArgumentsFailClosed()
{
    RequireFailure({"rl_sim_LW", "--plot-rate-hz"}, "requires an integer");
    RequireFailure(
        {"rl_sim_LW", "--enable-plot", "--plot-rate-hz", "fast"},
        "Invalid --plot-rate-hz");
    for (const char* rate : {"0", "-1", "201"})
    {
        RequireFailure(
            {"rl_sim_LW", "--enable-plot", "--plot-rate-hz", rate},
            "between 1 and 200");
    }
    RequireFailure(
        {"rl_sim_LW", "--plot-rate-hz", "50"},
        "requires --enable-plot");
    RequireFailure(
        {"rl_sim_LW",
         "--enable-plot",
         "--plot-rate-hz",
         "50",
         "--plot-rate-hz",
         "100"},
        "Conflicting --plot-rate-hz");
}
} // namespace

int main()
{
    try
    {
        TestPlotDefaultsOffAtOneHundredHertz();
        TestExplicitPlotRatesAndPeriods();
        TestInvalidPlotArgumentsFailClosed();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "test_lw_sim_plot_config failed: "
                  << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "test_lw_sim_plot_config passed" << std::endl;
    return EXIT_SUCCESS;
}
