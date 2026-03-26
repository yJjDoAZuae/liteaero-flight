#include <liteaero/autopilot/ControlHeading.hpp>
#include <liteaero/nav/KinematicStateUtil.hpp>

namespace KSU = liteaero::nav::KinematicStateUtil;

using namespace liteaero::autopilot;

float ControlHeading::step(float command, const liteaero::nav::KinematicStateSnapshot& state)
{
    controller_.setUnwrapInputs(true);
    return controller_.step(command, KSU::heading_rad(state), KSU::euler_rates_rad_s(state)(2));
}

void ControlHeading::reset(float command, const liteaero::nav::KinematicStateSnapshot& state)
{
    controller_.setUnwrapInputs(true);
    controller_.reset(command, KSU::heading_rad(state), 0.0f);
}
