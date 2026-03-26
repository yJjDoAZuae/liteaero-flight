#pragma once

#include <liteaero/autopilot/ControlLoop.hpp>

namespace liteaero::autopilot {

class ControlLoadFactor : public ControlLoop {

    void configure();
    void configure(nlohmann::json config);
    float step(float load_factor_command, const liteaero::nav::KinematicStateSnapshot& state);
    void reset(float load_factor_command, const liteaero::nav::KinematicStateSnapshot& state);

};

} // namespace liteaero::autopilot
