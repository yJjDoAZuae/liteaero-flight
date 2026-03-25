#define _USE_MATH_DEFINES
#include <cmath>
#include <liteaero/control/Unwrap.hpp>

using namespace liteaero::control;

static float wrapToPi(float angle)
{
    float a = std::fmod(angle + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI));
    if (a < 0.0f) a += 2.0f * static_cast<float>(M_PI);
    return a - static_cast<float>(M_PI);
}

float Unwrap::onStep(float u) {
    float y = ref_ + wrapToPi(u - ref_);
    ref_ = y;
    return y;
}

float Unwrap::step(float u, float ref) {
    ref_ = ref;
    return SisoElement::step(u);
}

nlohmann::json Unwrap::onSerializeJson() const {
    return {
        {"in",  in_},
        {"out", out_},
        {"ref", ref_}
    };
}

void Unwrap::onDeserializeJson(const nlohmann::json& state) {
    in_  = state.at("in").get<float>();
    out_ = state.at("out").get<float>();
    ref_ = state.contains("ref") ? state.at("ref").get<float>() : out_;
}
