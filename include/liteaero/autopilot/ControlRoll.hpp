#pragma once

#include <liteaero/autopilot/ControlLoop.hpp>

namespace liteaero::autopilot {

// input command is roll angle about velocity vector
// output command is roll rate about velocity vector

class ControlRoll : public ControlLoop {

    void configure();
    void configure(nlohmann::json config);
    float step(float command, const liteaero::nav::KinematicStateSnapshot& state);
    void reset(float command, const liteaero::nav::KinematicStateSnapshot& state);

};

} // namespace liteaero::autopilot
