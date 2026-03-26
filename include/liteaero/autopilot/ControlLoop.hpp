#pragma once

#include <liteaero/control/SISOPIDFF.hpp>
#include <liteaero/nav/KinematicStateSnapshot.hpp>
#include <nlohmann/json.hpp>

namespace liteaero::autopilot {

class ControlLoop {

public:

    virtual void configure() = 0;
    virtual void configure(nlohmann::json config) = 0;
    virtual float step(float command, const liteaero::nav::KinematicStateSnapshot& state) = 0;
    virtual void reset(float command, const liteaero::nav::KinematicStateSnapshot& state) = 0;

    float output()      const { return controller_.output(); }
    float command()     const { return controller_.command(); }
    float measurement() const { return controller_.measurement(); }
    float error()       const { return controller_.error(); }
    float feedForward() const { return controller_.feedForward(); }

protected:
    liteaero::control::SISOPIDFF controller_;

};

} // namespace liteaero::autopilot
