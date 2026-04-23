#include "internal/model_infer.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <utility>

#include "internal/model_adapter.hpp"
#include "internal/model_runtime.hpp"

namespace rmcs_laser_guidance {

struct ModelInfer::Details {
    explicit Details(InferenceConfig config_in)
        : config(std::move(config_in))
        , runtime_enabled(model_runtime_enabled_in_build())
        , runtime(config.model_path)
        , adapter(make_default_model_adapter()) {
        initialize();
    }

    auto initialize() -> void {
        if (!runtime_enabled) {
            message =
                "model backend requires ONNX Runtime support; reconfigure with "
                "-DRMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME=ON";
            return;
        }

        if (config.model_path.empty()) {
            message = "model backend requires inference.model_path to be set";
            return;
        }

        if (!std::filesystem::exists(config.model_path)) {
            message = "configured ONNX model does not exist: " + config.model_path.string();
            return;
        }

        message = runtime.load();
        if (!message.empty())
            return;

        startup_ready = true;
    }

    auto make_base_result() const -> ModelInferResult {
        return {
            .enabled = runtime_enabled,
            .success = false,
            .contract_supported = false,
            .observation = { },
            .candidates = { },
            .inputs = runtime.input_values(),
            .outputs = runtime.output_values(),
            .message = message,
        };
    }

    InferenceConfig config;
    bool runtime_enabled = false;
    bool startup_ready = false;
    std::string message { };
    ModelRuntime runtime;
    std::unique_ptr<ModelAdapter> adapter;
};

ModelInfer::ModelInfer(InferenceConfig config)
    : details_(std::make_unique<Details>(std::move(config))) { }

ModelInfer::~ModelInfer() = default;

ModelInfer::ModelInfer(ModelInfer&&) noexcept = default;

auto ModelInfer::operator=(ModelInfer&&) noexcept -> ModelInfer& = default;

auto ModelInfer::infer(const Frame& frame) const -> ModelInferResult {
    if (!details_->startup_ready)
        return details_->make_base_result();

    auto result = details_->make_base_result();

    if (frame.image.empty()) {
        result.message = "model backend received an empty frame";
        return result;
    }

    const auto adapter_result = details_->adapter->adapt(frame, details_->runtime);
    result.success = adapter_result.success;
    result.contract_supported = adapter_result.contract_supported;
    result.observation = adapter_result.observation;
    result.candidates = adapter_result.candidates;
    result.message = adapter_result.message;
    return result;
}

} // namespace rmcs_laser_guidance
