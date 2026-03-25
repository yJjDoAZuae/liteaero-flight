#pragma once

namespace liteaero::nav {

/// Minimal WGS84 geodetic position — data only.
///
/// Latitude and longitude are double-precision because single-precision float
/// loses ~1 m positional resolution at mid-latitudes. Altitude is float because
/// 1 cm altitude resolution is more than sufficient.
///
/// All derived quantities (radii of curvature, ECEF, gravity, qne) are free
/// functions in namespace liteaero::nav::WGS84.
struct GeodeticPosition {
    double latitude_rad  = 0.0;   ///< Geodetic latitude, radians [-π/2, π/2]
    double longitude_rad = 0.0;   ///< Longitude, radians [-π, π]
    float  altitude_m    = 0.0f;  ///< Height above WGS84 ellipsoid, meters
};

} // namespace liteaero::nav
