#include "inference_runtime.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
namespace fs = std::filesystem;
using Bytes = std::vector<std::uint8_t>;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
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

void appendVarint(Bytes& output, std::uint64_t value)
{
    while (value >= 0x80U)
    {
        output.push_back(
            static_cast<std::uint8_t>((value & 0x7fU) | 0x80U));
        value >>= 7U;
    }
    output.push_back(static_cast<std::uint8_t>(value));
}

void appendTag(Bytes& output, std::uint32_t field, std::uint8_t wire_type)
{
    appendVarint(output, (static_cast<std::uint64_t>(field) << 3U)
        | wire_type);
}

void appendVarintField(Bytes& output, std::uint32_t field, std::uint64_t value)
{
    appendTag(output, field, 0U);
    appendVarint(output, value);
}

void appendBytesField(Bytes& output, std::uint32_t field, const Bytes& value)
{
    appendTag(output, field, 2U);
    appendVarint(output, value.size());
    output.insert(output.end(), value.begin(), value.end());
}

void appendStringField(
    Bytes& output,
    std::uint32_t field,
    const std::string& value)
{
    appendBytesField(output, field, Bytes(value.begin(), value.end()));
}

struct Dimension
{
    std::int64_t value = 0;
    std::string parameter;
};

Dimension fixed(std::int64_t value)
{
    return {value, {}};
}

Dimension dynamic(const std::string& parameter)
{
    return {0, parameter};
}

Bytes makeValueInfo(
    const std::string& name,
    std::uint64_t element_type,
    const std::vector<Dimension>& dimensions)
{
    Bytes shape;
    for (const Dimension& dimension : dimensions)
    {
        Bytes encoded_dimension;
        if (dimension.parameter.empty())
        {
            appendVarintField(
                encoded_dimension,
                1U,
                static_cast<std::uint64_t>(dimension.value));
        }
        else
        {
            appendStringField(encoded_dimension, 2U, dimension.parameter);
        }
        appendBytesField(shape, 1U, encoded_dimension);
    }

    Bytes tensor_type;
    appendVarintField(tensor_type, 1U, element_type);
    appendBytesField(tensor_type, 2U, shape);

    Bytes type;
    appendBytesField(type, 1U, tensor_type);

    Bytes value_info;
    appendStringField(value_info, 1U, name);
    appendBytesField(value_info, 2U, type);
    return value_info;
}

Bytes makeIdentityNode(
    const std::string& input,
    const std::string& output)
{
    Bytes node;
    appendStringField(node, 1U, input);
    appendStringField(node, 2U, output);
    appendStringField(node, 4U, "Identity");
    return node;
}

Bytes makeIntegerAttribute(const std::string& name, std::int64_t value)
{
    Bytes attribute;
    appendStringField(attribute, 1U, name);
    appendVarintField(attribute, 3U, static_cast<std::uint64_t>(value));
    appendVarintField(attribute, 20U, 2U);
    return attribute;
}

Bytes makeCompressNode()
{
    Bytes node;
    appendStringField(node, 1U, "input");
    appendStringField(node, 1U, "condition");
    appendStringField(node, 2U, "output");
    appendStringField(node, 4U, "Compress");
    appendBytesField(node, 5U, makeIntegerAttribute("axis", 1));
    return node;
}

Bytes makeDynamicOutputModel()
{
    Bytes condition;
    appendVarintField(condition, 1U, 3U);
    appendVarintField(condition, 2U, 9U);
    appendStringField(condition, 8U, "condition");
    appendBytesField(condition, 9U, Bytes{1U, 0U, 1U});

    Bytes graph;
    appendBytesField(graph, 1U, makeCompressNode());
    appendStringField(graph, 2U, "dynamic_output_contract");
    appendBytesField(graph, 5U, condition);
    appendBytesField(
        graph,
        11U,
        makeValueInfo("input", 1U, {fixed(1), fixed(3)}));
    appendBytesField(
        graph,
        12U,
        makeValueInfo("output", 1U, {fixed(1), dynamic("selected")}));

    Bytes model;
    appendVarintField(model, 1U, 8U);
    appendStringField(model, 2U, "rl_sar_test");
    appendBytesField(model, 7U, graph);
    Bytes opset;
    appendVarintField(opset, 2U, 13U);
    appendBytesField(model, 8U, opset);
    return model;
}

Bytes makeIdentityModel(
    const std::vector<Dimension>& dimensions,
    std::uint64_t element_type = 1U,
    std::size_t input_count = 1U,
    std::size_t output_count = 1U)
{
    Bytes graph;
    for (std::size_t index = 0; index < output_count; ++index)
    {
        appendBytesField(
            graph,
            1U,
            makeIdentityNode(
                "input",
                index == 0U ? "output" : "output_" + std::to_string(index)));
    }
    appendStringField(graph, 2U, "inference_runtime_contract");
    for (std::size_t index = 0; index < input_count; ++index)
    {
        appendBytesField(
            graph,
            11U,
            makeValueInfo(
                index == 0U ? "input" : "unused_" + std::to_string(index),
                element_type,
                dimensions));
    }
    for (std::size_t index = 0; index < output_count; ++index)
    {
        appendBytesField(
            graph,
            12U,
            makeValueInfo(
                index == 0U ? "output" : "output_" + std::to_string(index),
                element_type,
                dimensions));
    }

    Bytes model;
    appendVarintField(model, 1U, 8U);
    appendStringField(model, 2U, "rl_sar_test");
    appendBytesField(model, 7U, graph);
    Bytes opset;
    appendVarintField(opset, 2U, 13U);
    appendBytesField(model, 8U, opset);
    return model;
}

class TemporaryDirectory
{
public:
    TemporaryDirectory()
    {
        const auto timestamp = std::chrono::steady_clock::now()
                                   .time_since_epoch()
                                   .count();
        path_ = fs::temp_directory_path()
            / ("rl-sar-inference-runtime-" + std::to_string(timestamp));
        require(fs::create_directory(path_), "failed to create test directory");
    }

    ~TemporaryDirectory()
    {
        std::error_code error;
        fs::remove_all(path_, error);
    }

    const fs::path& path() const
    {
        return path_;
    }

private:
    fs::path path_;
};

fs::path writeModel(
    const fs::path& directory,
    const std::string& name,
    const Bytes& content)
{
    const fs::path path = directory / name;
    std::ofstream output(path, std::ios::binary);
    require(output.is_open(), "failed to open model fixture");
    output.write(
        reinterpret_cast<const char*>(content.data()),
        static_cast<std::streamsize>(content.size()));
    require(output.good(), "failed to write model fixture");
    return path;
}

void requireIdentityInference(
    InferenceRuntime::Model& model,
    std::size_t element_count,
    float seed)
{
    std::vector<float> input(element_count);
    for (std::size_t index = 0; index < input.size(); ++index)
    {
        input[index] = seed + static_cast<float>(index) * 0.25F;
    }
    std::vector<float> output(element_count, 0.0F);
    const InferenceRuntime::TensorView input_view = {
        input.data(), input.size()};
    model.forwardInto(
        &input_view,
        1,
        {output.data(), output.size()});
    require(output == input, "concurrent identity output differs");
}

void testConcurrentModelLifecycle(const fs::path& directory)
{
    constexpr std::size_t model_count = 4U;
    std::array<fs::path, model_count> model_paths;
    for (std::size_t index = 0; index < model_count; ++index)
    {
        model_paths[index] = writeModel(
            directory,
            "concurrent_" + std::to_string(index) + ".onnx",
            makeIdentityModel(
                {fixed(1), fixed(static_cast<std::int64_t>(index + 2U))}));
    }

    for (std::size_t round = 0; round < 8U; ++round)
    {
        std::array<std::unique_ptr<InferenceRuntime::Model>, model_count>
            models;
        std::array<std::exception_ptr, model_count> failures;
        std::array<std::thread, model_count> workers;
        std::atomic<bool> start{false};

        for (std::size_t index = 0; index < model_count; ++index)
        {
            workers[index] = std::thread([&, index]() {
                while (!start.load(std::memory_order_acquire))
                {
                    std::this_thread::yield();
                }
                try
                {
                    auto model = InferenceRuntime::ModelFactory::load_model(
                        model_paths[index].string());
                    require(model != nullptr, "concurrent model load failed");
                    require(
                        model->input_metadata().front().shape
                            == std::vector<std::int64_t>(
                                {1, static_cast<std::int64_t>(index + 2U)}),
                        "concurrent model metadata crossed sessions");
                    requireIdentityInference(
                        *model,
                        index + 2U,
                        static_cast<float>(round * model_count + index));
                    models[index] = std::move(model);
                }
                catch (...)
                {
                    failures[index] = std::current_exception();
                }
            });
        }

        start.store(true, std::memory_order_release);
        for (std::thread& worker : workers)
        {
            worker.join();
        }
        for (const std::exception_ptr& failure : failures)
        {
            if (failure)
            {
                std::rethrow_exception(failure);
            }
        }

        std::exception_ptr first_inference_failure;
        std::exception_ptr second_inference_failure;
        std::thread destroy_first([&]() { models[1].reset(); });
        std::thread infer_first([&]() {
            try
            {
                requireIdentityInference(*models[0], 2U, 10.0F);
            }
            catch (...)
            {
                first_inference_failure = std::current_exception();
            }
        });
        std::thread destroy_second([&]() { models[3].reset(); });
        std::thread infer_second([&]() {
            try
            {
                requireIdentityInference(*models[2], 4U, 20.0F);
            }
            catch (...)
            {
                second_inference_failure = std::current_exception();
            }
        });
        destroy_first.join();
        infer_first.join();
        destroy_second.join();
        infer_second.join();
        if (first_inference_failure)
        {
            std::rethrow_exception(first_inference_failure);
        }
        if (second_inference_failure)
        {
            std::rethrow_exception(second_inference_failure);
        }

        requireIdentityInference(*models[0], 2U, 30.0F);
        requireIdentityInference(*models[2], 4U, 40.0F);
    }
}

void testStaticModelAndInputValidation(const fs::path& directory)
{
    const fs::path model_path = writeModel(
        directory,
        "static.onnx",
        makeIdentityModel({fixed(1), fixed(3)}));
    auto model = InferenceRuntime::ModelFactory::load_model(
        model_path.string());
    require(model != nullptr, "failed to load static model fixture");
    require(model->input_metadata().size() == 1U, "input metadata missing");
    require(model->output_metadata().size() == 1U, "output metadata missing");
    require(
        model->input_metadata().front().shape
            == std::vector<std::int64_t>({1, 3}),
        "static input shape differs");
    require(
        model->output_metadata().front().shape
            == std::vector<std::int64_t>({1, 3}),
        "static output shape differs");

    const std::vector<float> input{1.0F, -2.0F, 3.5F};
    require(model->forward({input}) == input, "identity output differs");
    std::vector<float> caller_output(3, 0.0F);
    const float* const caller_output_data = caller_output.data();
    const InferenceRuntime::TensorView input_view = {
        input.data(), input.size()};
    model->forwardInto(
        &input_view,
        1,
        {caller_output.data(), caller_output.size()});
    require(caller_output == input, "caller-owned identity output differs");
    require(
        caller_output.data() == caller_output_data,
        "caller-owned identity output storage was replaced");

    requireFailure(
        [&]() { static_cast<void>(model->forward({})); },
        "exactly one input tensor");
    requireFailure(
        [&]() {
            static_cast<void>(model->forward({input, input}));
        },
        "exactly one input tensor");
    requireFailure(
        [&]() {
            static_cast<void>(model->forward({{1.0F, 2.0F}}));
        },
        "element count must be 3, got 2");
    requireFailure(
        [&]() {
            static_cast<void>(model->forward({{1.0F, 2.0F, 3.0F, 4.0F}}));
        },
        "element count must be 3, got 4");
    requireFailure(
        [&]() {
            std::vector<float> short_output(2);
            model->forwardInto(
                &input_view,
                1,
                {short_output.data(), short_output.size()});
        },
        "output element count must be 3, got 2");
    requireFailure(
        [&]() {
            model->forwardInto(
                &input_view,
                1,
                {nullptr, 3});
        },
        "output tensor view is null");
    requireFailure(
        [&]() { model->forwardInto(nullptr, 1, {caller_output.data(), 3}); },
        "input tensor view is null");
    const InferenceRuntime::TensorView null_input{nullptr, 3};
    requireFailure(
        [&]() { model->forwardInto(&null_input, 1, {caller_output.data(), 3}); },
        "input tensor view is null");
    requireFailure(
        [&]() {
            std::vector<float> long_output(4);
            model->forwardInto(&input_view, 1, {long_output.data(), long_output.size()});
        },
        "output element count must be 3, got 4");
}

void testUnsupportedModelsFailDuringLoad(const fs::path& directory)
{
    struct Case
    {
        std::string name;
        Bytes model;
    };
    const std::vector<Case> cases{
        {"dynamic_batch.onnx",
         makeIdentityModel({dynamic("batch"), fixed(3)})},
        {"dynamic_feature.onnx",
         makeIdentityModel({fixed(1), dynamic("features")})},
        {"dynamic_output_feature.onnx", makeDynamicOutputModel()},
        {"batch_two.onnx", makeIdentityModel({fixed(2), fixed(3)})},
        {"rank_one.onnx", makeIdentityModel({fixed(3)})},
        {"int64.onnx", makeIdentityModel({fixed(1), fixed(3)}, 7U)},
        {"two_inputs.onnx",
         makeIdentityModel({fixed(1), fixed(3)}, 1U, 2U, 1U)},
        {"two_outputs.onnx",
         makeIdentityModel({fixed(1), fixed(3)}, 1U, 1U, 2U)},
    };

    for (const Case& test_case : cases)
    {
        const fs::path path = writeModel(
            directory, test_case.name, test_case.model);
        auto model = InferenceRuntime::ModelFactory::load_model(path.string());
        require(model == nullptr, "unsupported model loaded: " + test_case.name);
    }
}

void testFailedReloadClearsState(const fs::path& directory)
{
    const fs::path valid = writeModel(
        directory,
        "reload_static.onnx",
        makeIdentityModel({fixed(1), fixed(2)}));
    const fs::path invalid_input = writeModel(
        directory,
        "reload_dynamic.onnx",
        makeIdentityModel({dynamic("batch"), fixed(2)}));
    const fs::path invalid_output = writeModel(
        directory, "reload_dynamic_output.onnx", makeDynamicOutputModel());
    const fs::path replacement = writeModel(
        directory, "reload_replacement.onnx", makeIdentityModel({fixed(1), fixed(3)}));

    InferenceRuntime::ONNXModel model;
    for (const auto& invalid : {directory / "missing.onnx", invalid_input, invalid_output})
    {
        require(model.load(valid.string()), "initial static load failed");
        require(model.forward({{1.0F, 2.0F}}) == std::vector<float>({1.0F, 2.0F}),
                "initial inference differs");
        require(!model.load(invalid.string()), "invalid reload unexpectedly passed");
        require(!model.is_loaded(), "failed reload retained loaded state");
        require(model.input_metadata().empty(), "failed reload retained input metadata");
        require(model.output_metadata().empty(), "failed reload retained output metadata");
        requireFailure(
            [&]() { static_cast<void>(model.forward({{1.0F, 2.0F}})); },
            "Model not loaded");
        requireFailure(
            [&]() { model.forwardInto(nullptr, 0, {nullptr, 0}); },
            "Model not loaded");
        require(model.load(replacement.string()), "reload recovery failed");
        require(model.is_loaded(), "recovered model did not report loaded state");
        const std::vector<float> input{3.0F, -2.0F, 1.0F};
        require(model.forward({input}) == input, "recovered model retained stale caches");
    }
}
} // namespace

int main()
{
    try
    {
        TemporaryDirectory directory;
        testConcurrentModelLifecycle(directory.path());
        testStaticModelAndInputValidation(directory.path());
        testUnsupportedModelsFailDuringLoad(directory.path());
        testFailedReloadClearsState(directory.path());
    }
    catch (const std::exception& exception)
    {
        std::cerr << "test_inference_runtime failed: "
                  << exception.what() << std::endl;
        return 1;
    }
    std::cout << "test_inference_runtime passed" << std::endl;
    return 0;
}
