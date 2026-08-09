#include "lw_safety_policy.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void testDecisionMatrixIsCompleteAndOrdered()
{
    for (std::size_t index = 0; index < kLWSafetyDecisions.size(); ++index)
    {
        const auto event = static_cast<LWSafetyEvent>(index);
        const auto& decision = LWSafetyDecisionFor(event);
        require(decision.event == event, "safety matrix event order mismatch");
        require(decision.name != nullptr && decision.name[0] != '\0',
                "safety matrix event has no diagnostic name");
    }
}

void testSourceSpecificActions()
{
    const auto require_action = [](LWSafetyEvent event, LWSafetyAction action)
    {
        require(
            LWSafetyDecisionFor(event).action == action,
            std::string("unexpected action for ")
                + LWSafetyDecisionFor(event).name);
    };

    for (const auto event : {
             LWSafetyEvent::FeedbackParserError,
             LWSafetyEvent::TorqueLimitWarning})
    {
        require_action(event, LWSafetyAction::Observe);
    }
    for (const auto event : {
             LWSafetyEvent::JoystickUnavailable,
             LWSafetyEvent::JoystickLoopException,
             LWSafetyEvent::ControlTimingDegraded})
    {
        require_action(event, LWSafetyAction::InhibitInput);
    }
    for (const auto event : {
             LWSafetyEvent::InferenceLoopException,
             LWSafetyEvent::PolicyActionInvalid,
             LWSafetyEvent::PolicyOutputInvalid,
             LWSafetyEvent::PolicyConfigurationInvalid,
             LWSafetyEvent::PolicyOutputUnavailable})
    {
        require_action(event, LWSafetyAction::PassiveDamping);
    }
    require_action(
        LWSafetyEvent::MotorHardwareFault,
        LWSafetyAction::HardDisable);
    for (const auto event : {
             LWSafetyEvent::ControlLoopException,
             LWSafetyEvent::UnknownLoopException,
             LWSafetyEvent::ControlTimingFatal,
             LWSafetyEvent::SensorDataStale,
             LWSafetyEvent::WaitingDisableIncomplete,
             LWSafetyEvent::FeedbackInvalid,
             LWSafetyEvent::FsmStateMissing,
             LWSafetyEvent::AttitudeLimitExceeded,
             LWSafetyEvent::RobotCommandInvalid,
             LWSafetyEvent::NullRobotCommand,
             LWSafetyEvent::FeedbackReadFailed,
             LWSafetyEvent::ControlCommandIncomplete})
    {
        require_action(event, LWSafetyAction::HardDisableAndShutdown);
    }
    for (const auto event : {
             LWSafetyEvent::StartupSerialInitializationFailed,
             LWSafetyEvent::StartupInitialDisableIncomplete,
             LWSafetyEvent::StartupLoopStartFailed})
    {
        require_action(event, LWSafetyAction::AbortStartup);
    }
    require_action(
        LWSafetyEvent::NormalShutdown,
        LWSafetyAction::OrderlyShutdown);

    require(
        LWSafetyDecisionFor(LWSafetyEvent::JoystickLoopException).action
            == LWSafetyAction::InhibitInput,
        "joystick exception incorrectly hard-disabled the robot");
    require(
        LWSafetyDecisionFor(LWSafetyEvent::InferenceLoopException).action
            == LWSafetyAction::PassiveDamping,
        "inference exception did not request controlled damping");
    require(
        LWSafetyDecisionFor(LWSafetyEvent::ControlLoopException).action
            == LWSafetyAction::HardDisableAndShutdown,
        "control exception did not retain fatal protection");
    require(
        LWSafetyDecisionFor(LWSafetyEvent::MotorHardwareFault).action
            == LWSafetyAction::HardDisable,
        "motor fault did not separate hard-disable from ROS shutdown");
    require(
        LWSafetyDecisionFor(LWSafetyEvent::NormalShutdown).action
            == LWSafetyAction::OrderlyShutdown,
        "normal shutdown was treated as a runtime fault");
}

void testSupervisorLatchesFallbackAndEscalates()
{
    LWSafetySupervisor supervisor;
    supervisor.report(LWSafetyEvent::PolicyOutputUnavailable);
    require(supervisor.controlledFallbackLatched(),
            "policy-output fault did not latch controlled fallback");
    auto snapshot = supervisor.snapshot();
    require(snapshot.highest_severity == LWSafetySeverity::ControlledFallback,
            "controlled fallback severity was not recorded");
    const std::uint64_t fallback_sequence = snapshot.sequence;
    supervisor.report(LWSafetyEvent::PolicyOutputUnavailable);
    require(supervisor.snapshot().sequence == fallback_sequence,
            "repeated identical safety event spammed diagnostics");

    supervisor.report(LWSafetyEvent::ControlCommandIncomplete);
    snapshot = supervisor.snapshot();
    require(snapshot.highest_severity == LWSafetySeverity::FatalShutdown,
            "failed fallback send did not escalate to fatal shutdown");

    supervisor.report(LWSafetyEvent::FeedbackParserError);
    require(
        supervisor.snapshot().highest_severity
            == LWSafetySeverity::FatalShutdown,
        "later diagnostic event weakened a fatal latch");
}
} // namespace

int main()
{
    try
    {
        testDecisionMatrixIsCompleteAndOrdered();
        testSourceSpecificActions();
        testSupervisorLatchesFallbackAndEscalates();
        std::cout << "LW safety policy tests passed" << std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW safety policy tests failed: "
                  << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
