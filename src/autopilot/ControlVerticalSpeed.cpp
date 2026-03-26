#include <liteaero/autopilot/ControlVerticalSpeed.hpp>
#include <liteaero/nav/KinematicStateUtil.hpp>
#include <Eigen/Dense>
#include <cmath>

namespace KSU = liteaero::nav::KinematicStateUtil;

using namespace liteaero::autopilot;

namespace {

float velWind_horiz(const liteaero::nav::KinematicStateSnapshot& state)
{
    // airmass-relative velocity expressed in NED frame
    Eigen::Vector3f velWind_ned_mps = state.q_nw.toRotationMatrix() * KSU::velocity_wind_mps(state);
    return hypot(velWind_ned_mps(0), velWind_ned_mps(1));
}

} // namespace

float ControlVerticalSpeed::step(float command, const liteaero::nav::KinematicStateSnapshot& state)
{
    controller_.setUnwrapInputs(false);

    float gamma_command     = atan2(command, velWind_horiz(state));  // positive up
    float gamma_measurement = atan2(-state.velocity_ned_mps(2), velWind_horiz(state));
    float gamma_dot         = KSU::euler_rates_rad_s(state)(1)
                              - state.alpha_dot_rad_s * cos(KSU::roll_rad(state));

    float gamma_rate_command = controller_.step(gamma_command, gamma_measurement, gamma_dot);

    return gamma_rate_command * KSU::airspeed_mps(state);
}

void ControlVerticalSpeed::reset(float command, const liteaero::nav::KinematicStateSnapshot& state)
{
    controller_.setUnwrapInputs(false);

    float gamma_command     = atan2(command, velWind_horiz(state));  // positive up
    float gamma_measurement = atan2(-state.velocity_ned_mps(2), velWind_horiz(state));
    controller_.reset(gamma_command, gamma_measurement, 0.0f);
}
