#include "internal/model_adapter.hpp"

#include <memory>

#include "internal/model_runtime.hpp"

namespace rmcs_laser_guidance {
namespace {

class UnsupportedModelAdapter final : public ModelAdapter {
public:
    auto adapt(const Frame& frame, const ModelRuntime& runtime) const -> ModelAdapterResult override {
        (void)frame;

        ModelAdapterResult result;
        result.message = "model output contract is not adapted for this build";

        if (!runtime.input_values().empty() || !runtime.output_values().empty()) {
            result.message += " (inputs=" + std::to_string(runtime.input_values().size());
            result.message += ", outputs=" + std::to_string(runtime.output_values().size()) + ")";
        }

        return result;
    }
};

} // namespace

auto make_default_model_adapter() -> std::unique_ptr<ModelAdapter> {
    return std::make_unique<UnsupportedModelAdapter>();
}

} // namespace rmcs_laser_guidance
