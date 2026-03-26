#include <liteaero/autopilot/ControlAltitude.hpp>

using namespace liteaero::autopilot;

float ControlAltitude::step(float command, const liteaero::nav::KinematicStateSnapshot& state)
{
    controller_.setUnwrapInputs(true);
    return controller_.step(command, state.position.altitude_m, -state.velocity_ned_mps(2));
}

void ControlAltitude::reset(float command, const liteaero::nav::KinematicStateSnapshot& state)
{
    controller_.setUnwrapInputs(true);
    controller_.reset(command, state.position.altitude_m, 0.0f);
}
