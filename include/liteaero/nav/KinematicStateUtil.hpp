#pragma once

#include <liteaero/nav/KinematicStateSnapshot.hpp>
#include <liteaero/nav/WGS84.hpp>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <cstdint>
#include <vector>

namespace liteaero::nav {

// ---------------------------------------------------------------------------
// Auxiliary types returned by KinematicStateUtil functions
// ---------------------------------------------------------------------------

/// Plane-of-motion frame: x-axis aligned with airmass-relative velocity,
/// y-axis toward the center of curvature, z-axis completing the right-hand triad.
struct PlaneOfMotion {
    Eigen::Quaternionf q_np = Eigen::Quaternionf::Identity();  ///< POM-to-NED rotation
};

/// Instantaneous turn circle in the plane of motion.
struct TurnCircle {
    PlaneOfMotion pom;
    Eigen::Vector3f turn_center_delta_ned_m = Eigen::Vector3f::Zero();
    ///< Vector from aircraft position to turn center, NED frame, meters
};

} // namespace liteaero::nav

namespace liteaero::nav::KinematicStateUtil {

// ---------------------------------------------------------------------------
// Orientation
// ---------------------------------------------------------------------------

/// Body-to-NED rotation. Derived from q_nw, alpha_rad, beta_rad.
/// q_nb = q_nw · Ry(alpha) · Rz(-beta)
Eigen::Quaternionf q_nb(const KinematicStateSnapshot& s);

/// Stability-to-NED rotation: q_ns = q_nw · Ry(alpha)
Eigen::Quaternionf q_ns(const KinematicStateSnapshot& s);

/// Local-Level-to-NED rotation: q_nl = qne(position).cast<float>()
Eigen::Quaternionf q_nl(const KinematicStateSnapshot& s);


// ---------------------------------------------------------------------------
// Euler angles (derived; subject to singularity at pitch = ±90°)
// ---------------------------------------------------------------------------

/// Roll angle, radians. Extracted from q_nb via ZYX decomposition.
float roll_rad(const KinematicStateSnapshot& s);

/// Pitch angle, radians. Singularity at ±π/2.
float pitch_rad(const KinematicStateSnapshot& s);

/// Heading/yaw angle, radians [0, 2π).
float heading_rad(const KinematicStateSnapshot& s);

/// Euler rates [roll_rate, pitch_rate, heading_rate], rad/s.
/// Returns zero vector when pitch is within 1e-6 rad of ±π/2 (gimbal-lock guard).
Eigen::Vector3f euler_rates_rad_s(const KinematicStateSnapshot& s);


// ---------------------------------------------------------------------------
// Frame-transformed velocities
// ---------------------------------------------------------------------------

/// Ground velocity in body frame, m/s.
Eigen::Vector3f velocity_body_mps(const KinematicStateSnapshot& s);

/// Ground velocity in stability frame, m/s.
Eigen::Vector3f velocity_stab_mps(const KinematicStateSnapshot& s);

/// Airmass-relative velocity in wind frame, m/s.
Eigen::Vector3f velocity_wind_mps(const KinematicStateSnapshot& s);

/// Airspeed magnitude (airmass-relative speed), m/s.
float airspeed_mps(const KinematicStateSnapshot& s);


// ---------------------------------------------------------------------------
// Frame-transformed accelerations
// ---------------------------------------------------------------------------

/// Acceleration in body frame, m/s².
Eigen::Vector3f acceleration_body_mps2(const KinematicStateSnapshot& s);

/// Acceleration in wind frame, m/s².
Eigen::Vector3f acceleration_wind_mps2(const KinematicStateSnapshot& s);


// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Crab angle (heading of ground-track minus heading of airmass-relative velocity), radians.
float crab_rad(const KinematicStateSnapshot& s);

/// Crab angle rate, rad/s.
float crab_rate_rad_s(const KinematicStateSnapshot& s);

/// Plane of motion from airmass-relative velocity / acceleration curvature.
PlaneOfMotion plane_of_motion(const KinematicStateSnapshot& s);

/// Instantaneous turn circle from centripetal acceleration.
TurnCircle turn_circle(const KinematicStateSnapshot& s);


// ---------------------------------------------------------------------------
// Euler rate / body rate conversions (static utilities)
// ---------------------------------------------------------------------------

/// Convert Euler rates to body angular rates.
Eigen::Vector3f euler_rates_to_body_rates(const Eigen::Vector3f& euler_rad,
                                           const Eigen::Vector3f& euler_rates_rad_s);

/// Convert body angular rates to Euler rates. Returns zero at gimbal lock (pitch ≈ ±π/2).
Eigen::Vector3f body_rates_to_euler_rates(const Eigen::Vector3f& euler_rad,
                                           const Eigen::Vector3f& body_rates_rps);


// ---------------------------------------------------------------------------
// Serialization (free functions — value type has no methods)
// ---------------------------------------------------------------------------

nlohmann::json           serializeJson(const KinematicStateSnapshot& s);
KinematicStateSnapshot   deserializeJson(const nlohmann::json& j);

std::vector<uint8_t>     serializeProto(const KinematicStateSnapshot& s);
KinematicStateSnapshot   deserializeProto(const std::vector<uint8_t>& bytes);

} // namespace liteaero::nav::KinematicStateUtil
