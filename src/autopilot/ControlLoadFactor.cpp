#include <liteaero/autopilot/ControlLoadFactor.hpp>
#include <liteaero/nav/KinematicStateUtil.hpp>

namespace KSU = liteaero::nav::KinematicStateUtil;

using namespace liteaero::autopilot;

static float constexpr g = 9.81f;

float ControlLoadFactor::step(float load_factor_command, const liteaero::nav::KinematicStateSnapshot& state)
{
    return controller_.step(load_factor_command, -KSU::acceleration_wind_mps2(state)(2) / g);
}

void ControlLoadFactor::reset(float load_factor_command, const liteaero::nav::KinematicStateSnapshot& state)
{
    controller_.reset(load_factor_command, -KSU::acceleration_wind_mps2(state)(2) / g, 0.0f);
    controller_.integrator().resetTo(0.0f);
}
