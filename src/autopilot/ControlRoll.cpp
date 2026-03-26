#include <liteaero/autopilot/ControlRoll.hpp>
#include <liteaero/nav/KinematicStateUtil.hpp>
#include <Eigen/Dense>

namespace KSU = liteaero::nav::KinematicStateUtil;

using namespace liteaero::autopilot;

float ControlRoll::step(float command, const liteaero::nav::KinematicStateSnapshot& state)
{
    controller_.setUnwrapInputs(true);

    // ZYX Euler decomposition of Wind-to-NED rotation: [yaw, pitch, roll]
    Eigen::Vector3f eulersWind = state.q_nw.toRotationMatrix().eulerAngles(2, 1, 0);

    // use roll Euler of Wind frame w.r.t. NED and the roll rate of the Wind frame
    return controller_.step(command, eulersWind(2), state.roll_rate_wind_rad_s);
}

void ControlRoll::reset(float command, const liteaero::nav::KinematicStateSnapshot& state)
{
    controller_.setUnwrapInputs(true);

    // ZYX Euler decomposition of Wind-to-NED rotation: [yaw, pitch, roll]
    Eigen::Vector3f eulersWind = state.q_nw.toRotationMatrix().eulerAngles(2, 1, 0);

    controller_.reset(command, eulersWind(2), 0.0f);
}
