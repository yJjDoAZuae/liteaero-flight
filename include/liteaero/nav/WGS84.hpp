#pragma once

#include <liteaero/nav/GeodeticPosition.hpp>
#include <Eigen/Geometry>

namespace liteaero::nav::WGS84 {

// ---------------------------------------------------------------------------
// Defining ellipsoid constants (WGS84 standard)
// ---------------------------------------------------------------------------

inline constexpr double kA_m       = 6378137.0;          ///< Semi-major axis, m
inline constexpr double kFinv      = 298.257223563;       ///< Inverse flattening
inline constexpr double kGM        = 3.986004418e14;      ///< Gravitational constant, m³/s²
inline constexpr double kOmega_rps = 7.2921151467e-5;     ///< Earth rotation rate, rad/s

// ---------------------------------------------------------------------------
// Radii of curvature
// ---------------------------------------------------------------------------

/// Meridional radius of curvature (north-south), m.
double meridionalRadius(double latitude_rad);

/// Prime vertical radius of curvature, m.
double primeVerticalRadius(double latitude_rad);

/// Effective radius for north motion (= meridional radius), m.
double northRadius(const GeodeticPosition& p);

/// Effective radius for east motion (= prime vertical * cos(lat)), m.
double eastRadius(const GeodeticPosition& p);

// ---------------------------------------------------------------------------
// Kinematic rates
// ---------------------------------------------------------------------------

/// Geodetic latitude rate, rad/s.
double latitudeRate_rad_s(const GeodeticPosition& p, float v_north_mps);

/// Longitude rate, rad/s.
double longitudeRate_rad_s(const GeodeticPosition& p, float v_east_mps);

/// Angular rate of NED frame w.r.t. ECEF due to transport over ellipsoid,
/// expressed in NED frame, rad/s.
Eigen::Vector3d transportRate(const GeodeticPosition& p, float v_north_mps, float v_east_mps);

// ---------------------------------------------------------------------------
// Gravity and Earth rate
// ---------------------------------------------------------------------------

/// Gravity magnitude (Somigliana model), m/s².
double gravity_mps2(const GeodeticPosition& p);

/// Earth rotation rate expressed in NED frame, rad/s.
Eigen::Vector3d omega_ie_n(const GeodeticPosition& p);

// ---------------------------------------------------------------------------
// ECEF conversions
// ---------------------------------------------------------------------------

/// Convert geodetic position to ECEF, m.
Eigen::Vector3d toECEF(const GeodeticPosition& p);

/// Convert ECEF position to geodetic (Newton-Raphson iteration).
GeodeticPosition fromECEF(const Eigen::Vector3d& ecef);

// ---------------------------------------------------------------------------
// NED-to-ECEF rotation
// ---------------------------------------------------------------------------

/// NED-to-ECEF rotation quaternion (q_ne: maps vectors from NED to ECEF).
Eigen::Quaterniond qne(const GeodeticPosition& p);

/// Recover geodetic position from q_ne quaternion.
GeodeticPosition fromQne(const Eigen::Quaterniond& q_ne);

} // namespace liteaero::nav::WGS84
