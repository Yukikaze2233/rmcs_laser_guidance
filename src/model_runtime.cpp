#include "internal/model_runtime.hpp"

#include <memory>
#include <sstream>
#include <string>
#include <utility>

#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>
#endif

namespace rmcs_laser_guidance {

#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
namespace {

auto tensor_element_type_name(const ONNXTensorElementDataType type) -> std::string {
    switch (type) {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
        return "float32";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
        return "uint8";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
        return "int8";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
        return "uint16";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
        return "int16";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
        return "int32";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
        return "int64";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
        return "float64";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:
        return "bool";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
        return "float16";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16:
        return "bfloat16";
    default:
        return "unknown";
    }
}

auto extract_value_info(
    Ort::Session& session, Ort::AllocatorWithDefaultOptions& allocator, const bool is_input)
    -> std::vector<ModelValueInfo> {
    const std::size_t count = is_input ? session.GetInputCount() : session.GetOutputCount();
    std::vector<ModelValueInfo> values;
    values.reserve(count);

    for (std::size_t index = 0; index < count; ++index) {
        Ort::AllocatedStringPtr name = is_input
                                           ? session.GetInputNameAllocated(index, allocator)
                                           : session.GetOutputNameAllocated(index, allocator);
        Ort::TypeInfo type_info =
            is_input ? session.GetInputTypeInfo(index) : session.GetOutputTypeInfo(index);
        Ort::TensorTypeAndShapeInfo tensor_info = type_info.GetTensorTypeAndShapeInfo();

        ModelValueInfo value;
        value.name = name ? name.get() : "";
        value.shape = tensor_info.GetShape();
        value.element_type = tensor_element_type_name(tensor_info.GetElementType());
        values.push_back(std::move(value));
    }

    return values;
}

} // namespace

struct ModelRuntime::Details {
    Details()
        : env(ORT_LOGGING_LEVEL_WARNING, "rmcs_laser_guidance")
        , session_options() {
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    }

    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session { };
    std::vector<ModelValueInfo> inputs { };
    std::vector<ModelValueInfo> outputs { };
};

#else

struct ModelRuntime::Details {
    std::vector<ModelValueInfo> inputs { };
    std::vector<ModelValueInfo> outputs { };
};

#endif

ModelRuntime::ModelRuntime(std::filesystem::path model_path)
    : model_path_(std::move(model_path))
    , details_(std::make_unique<Details>()) { }

ModelRuntime::~ModelRuntime() = default;

ModelRuntime::ModelRuntime(ModelRuntime&&) noexcept = default;

auto ModelRuntime::operator=(ModelRuntime&&) noexcept -> ModelRuntime& = default;

auto ModelRuntime::load() -> std::string {
#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
    if (details_->session)
        return {};

    try {
        details_->session = std::make_unique<Ort::Session>(
            details_->env, model_path_.string().c_str(), details_->session_options);
        Ort::AllocatorWithDefaultOptions allocator;
        details_->inputs = extract_value_info(*details_->session, allocator, true);
        details_->outputs = extract_value_info(*details_->session, allocator, false);
        return {};
    } catch (const Ort::Exception& e) {
        std::ostringstream oss;
        oss << "failed to load ONNX model '" << model_path_.string() << "': " << e.what();
        return oss.str();
    }
#else
    return "model backend was built without ONNX Runtime support";
#endif
}

auto ModelRuntime::is_loaded() const noexcept -> bool {
#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
    return static_cast<bool>(details_->session);
#else
    return false;
#endif
}

auto ModelRuntime::model_path() const noexcept -> const std::filesystem::path& {
    return model_path_;
}

auto ModelRuntime::input_values() const noexcept -> const std::vector<ModelValueInfo>& {
    return details_->inputs;
}

auto ModelRuntime::output_values() const noexcept -> const std::vector<ModelValueInfo>& {
    return details_->outputs;
}

auto model_runtime_enabled_in_build() noexcept -> bool {
#if defined(RMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME)
    return true;
#else
    return false;
#endif
}

} // namespace rmcs_laser_guidance
