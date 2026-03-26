#pragma once

#include <liteaero/autopilot/ControlLoop.hpp>

namespace liteaero::autopilot {

class ControlHeadingRate : public ControlLoop {

    void configure();
    void configure(nlohmann::json config);
    float step(float heading_rate_command, const liteaero::nav::KinematicStateSnapshot& state);
    void reset(float heading_rate_command, const liteaero::nav::KinematicStateSnapshot& state);

};

} // namespace liteaero::autopilot
