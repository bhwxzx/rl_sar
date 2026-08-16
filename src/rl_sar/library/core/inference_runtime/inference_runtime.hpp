/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef INFERENCE_RUNTIME_HPP
#define INFERENCE_RUNTIME_HPP

#include <cstdint>
#include <vector>
#include <string>
#include <memory>
#include <filesystem>
#include <algorithm>
#include "logger.hpp"

#ifdef USE_ONNX
#include <onnxruntime_cxx_api.h>
#endif

namespace InferenceRuntime
{

enum class TensorElementType
{
    Unknown,
    Float32,
};

struct TensorMetadata
{
    std::string name;
    std::vector<int64_t> shape;
    TensorElementType element_type = TensorElementType::Unknown;
};

/**
 * @brief Model interface base class
 *
 * Defines the common interface for ONNX model loading and inference.
 */
class Model
{
public:
    virtual ~Model() = default;

    /**
     * @brief Load model file
     * @param model_path Model file path
     * @return Returns true if loading succeeds, false if it fails
     */
    virtual bool load(const std::string& model_path) = 0;

    /**
     * @brief Check if model is loaded
     * @return Returns true if loaded, false otherwise
     */
    virtual bool is_loaded() const = 0;

    /**
     * @brief Forward inference (single input, supports initializer list)
     * @param inputs Vector of input data vectors (usually single element)
     * @return Inference result vector
     */
    virtual std::vector<float> forward(const std::vector<std::vector<float>>& inputs) = 0;

    /**
     * @brief Get model type string
     * @return Model type ("onnx")
     */
    virtual std::string get_model_type() const = 0;

    /**
     * @brief Return immutable input tensor metadata when supported.
     */
    virtual const std::vector<TensorMetadata>& input_metadata() const;

    /**
     * @brief Return immutable output tensor metadata when supported.
     */
    virtual const std::vector<TensorMetadata>& output_metadata() const;
};

/**
 * @brief ONNX model implementation class
 *
 * Model inference implementation based on ONNX Runtime
 */
class ONNXModel : public Model
{
private:
    bool loaded_ = false;               ///< Whether model is loaded
    std::string model_path_;            ///< Model file path

#ifdef USE_ONNX
    std::unique_ptr<Ort::Session> session_;                 ///< ONNX inference session
    std::unique_ptr<Ort::Env> env_;                         ///< ONNX runtime environment
    Ort::MemoryInfo memory_info_;                           ///< Memory information
    std::vector<std::string> input_node_names_;             ///< Input node names
    std::vector<std::string> output_node_names_;            ///< Output node names
    std::vector<std::vector<int64_t>> input_shapes_;        ///< Input shapes
    std::vector<std::vector<int64_t>> output_shapes_;       ///< Output shapes
#endif
    std::vector<TensorMetadata> input_metadata_;
    std::vector<TensorMetadata> output_metadata_;

public:
    ONNXModel();
    ~ONNXModel();

    bool load(const std::string& model_path) override;
    bool is_loaded() const override { return loaded_; }
    std::vector<float> forward(const std::vector<std::vector<float>>& inputs) override;
    std::string get_model_type() const override { return "onnx"; }
    const std::vector<TensorMetadata>& input_metadata() const override
    {
        return input_metadata_;
    }
    const std::vector<TensorMetadata>& output_metadata() const override
    {
        return output_metadata_;
    }

private:
#ifdef USE_ONNX
    /**
     * @brief Setup input/output node information
     */
    void setup_input_output_info();

    /**
     * @brief Extract data from ONNX outputs
     * @param outputs ONNX inference outputs
     * @return Extracted data vector
     */
    std::vector<float> extract_output_data(const std::vector<Ort::Value>& outputs);
#endif
};

/**
 * @brief Model factory class
 *
 * Responsible for creating and loading different types of models
 */
class ModelFactory
{
public:
    /**
     * @brief Model type enumeration
     */
    enum class ModelType
    {
        ONNX,   ///< ONNX model
        AUTO    ///< Automatically detect model type
    };

    /**
     * @brief Create model of specified type
     * @param type Model type
     * @return Model smart pointer
     */
    static std::unique_ptr<Model> create_model(ModelType type = ModelType::AUTO);

    /**
     * @brief Detect model type based on file path
     * @param model_path Model file path
     * @return Detected model type
     */
    static ModelType detect_model_type(const std::string& model_path);

    /**
     * @brief Load model file
     * @param model_path Model file path
     * @param type Model type (default: auto-detect)
     * @return Successfully loaded model smart pointer, returns nullptr on failure
     */
    static std::unique_ptr<Model> load_model(const std::string& model_path, ModelType type = ModelType::AUTO);
};

} // namespace InferenceRuntime

#endif // INFERENCE_RUNTIME_HPP
