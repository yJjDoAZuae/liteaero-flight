#include <liteaero/autopilot/ControlHeadingRate.hpp>
#include <liteaero/nav/KinematicStateUtil.hpp>
#include <Eigen/Dense>

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

float ControlHeadingRate::step(float heading_rate_command, const liteaero::nav::KinematicStateSnapshot& state)
{
    float lateral_accel_command     = velWind_horiz(state) * heading_rate_command;
    float lateral_accel_measurement = KSU::euler_rates_rad_s(state)(2) * velWind_horiz(state);

    return controller_.step(lateral_accel_command, lateral_accel_measurement);
}

void ControlHeadingRate::reset(float heading_rate_command, const liteaero::nav::KinematicStateSnapshot& state)
{
    float lateral_accel_command     = velWind_horiz(state) * heading_rate_command;
    float lateral_accel_measurement = KSU::euler_rates_rad_s(state)(2) * velWind_horiz(state);

    controller_.reset(lateral_accel_command, lateral_accel_measurement, 0.0f);
    controller_.integrator().resetTo(0.0f);
}
