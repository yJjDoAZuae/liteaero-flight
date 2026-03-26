#pragma once

#include <liteaero/autopilot/ControlLoop.hpp>

namespace liteaero::autopilot {

// input is a vertical speed command (positive local level up)
// output is a vertical acceleration command (positive local level up)

class ControlVerticalSpeed : public ControlLoop {

    void configure();
    void configure(nlohmann::json config);
    float step(float command, const liteaero::nav::KinematicStateSnapshot& state);
    void reset(float command, const liteaero::nav::KinematicStateSnapshot& state);

};

} // namespace liteaero::autopilot
