#define _USE_MATH_DEFINES
#include <liteaero/nav/WGS84.hpp>
#include <cmath>

// File-scope helpers — not part of the public API.

namespace {

// Derived ellipsoid constants
constexpr double kE2  = 2.0 / liteaero::nav::WGS84::kFinv
                      - 1.0 / (liteaero::nav::WGS84::kFinv * liteaero::nav::WGS84::kFinv);
[[maybe_unused]] const double kE = std::sqrt(kE2);
[[maybe_unused]] const double kB = liteaero::nav::WGS84::kA_m * (1.0 - 1.0 / liteaero::nav::WGS84::kFinv);

// Wrap angle to [-π, π].
static double wrapToPi(double angle_rad)
{
    double w = std::fmod(angle_rad + M_PI, 2.0 * M_PI);
    if (w < 0.0) { w += 2.0 * M_PI; }
    return w - M_PI;
}

// Normalize (lat, lon) so that |lat| <= π/2 and lon in [-π, π].
static void normalizeLatLon(double* lat, double* lon)
{
    *lat = wrapToPi(*lat);
    *lon = wrapToPi(*lon);

    if (std::fabs(*lat) > M_PI / 2.0) {
        *lat = (*lat > 0.0) ? (M_PI - *lat) : (-M_PI - *lat);
        *lon = wrapToPi(*lon + M_PI);
    }
}

// Build direction cosine matrix C_ne (NED-to-ECEF) from lat/lon.
static void latLon2Cne(double lat, double lon, Eigen::Matrix3d& Cne)
{
    const Eigen::Matrix3d Cnn0 =
        (Eigen::AngleAxisd(lat, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(-lon, Eigen::Vector3d::UnitX())).toRotationMatrix();

    Eigen::Matrix3d Cn0e;
    Cn0e << 0, 0, 1,
            0, 1, 0,
           -1, 0, 0;

    Cne = Cnn0 * Cn0e;
}

// Recover lat/lon from C_ne.
static void Cne2LatLon(const Eigen::Matrix3d& Cne, double* lat, double* lon)
{
    Eigen::Matrix3d Cn0e;
    Cn0e << 0, 0, 1,
            0, 1, 0,
           -1, 0, 0;

    Eigen::Matrix3d Cnn0 = Cne * Cn0e.transpose();
    Eigen::Vector3d euler = Cnn0.eulerAngles(2, 1, 0);  // [yaw, pitch, roll]
    *lat = euler.y();
    *lon = -euler.z();
    normalizeLatLon(lat, lon);
}

} // anonymous namespace

namespace liteaero::nav::WGS84 {

// ---------------------------------------------------------------------------
// Radii of curvature
// ---------------------------------------------------------------------------

double meridionalRadius(double latitude_rad)
{
    const double s  = std::sin(latitude_rad);
    const double d  = 1.0 - kE2 * s * s;
    return kA_m * (1.0 - kE2) / std::pow(d, 1.5);
}

double primeVerticalRadius(double latitude_rad)
{
    const double s = std::sin(latitude_rad);
    return kA_m / std::sqrt(1.0 - kE2 * s * s);
}

double northRadius(const GeodeticPosition& p)
{
    return meridionalRadius(p.latitude_rad);
}

double eastRadius(const GeodeticPosition& p)
{
    return primeVerticalRadius(p.latitude_rad) * std::cos(p.latitude_rad);
}

// ---------------------------------------------------------------------------
// Kinematic rates
// ---------------------------------------------------------------------------

double latitudeRate_rad_s(const GeodeticPosition& p, float v_north_mps)
{
    return static_cast<double>(v_north_mps) / (northRadius(p) + p.altitude_m);
}

double longitudeRate_rad_s(const GeodeticPosition& p, float v_east_mps)
{
    const double R = primeVerticalRadius(p.latitude_rad) + p.altitude_m;
    return static_cast<double>(v_east_mps) / (R * std::cos(p.latitude_rad));
}

// Skew radius in an arbitrary azimuth direction.
static double skewRadius(const GeodeticPosition& p, double azimuth_rad)
{
    const double N      = primeVerticalRadius(p.latitude_rad);
    const double M      = meridionalRadius(p.latitude_rad);
    const double sin_az = std::sin(azimuth_rad);
    const double cos_az = std::cos(azimuth_rad);
    return N * M / (N * cos_az * cos_az + M * sin_az * sin_az);
}

// Horizon rate: rate of change of horizon angle due to forward motion.
static double horizonRate(const GeodeticPosition& p, float v_north_mps, float v_east_mps)
{
    const double az     = std::atan2(v_east_mps, v_north_mps);
    const double Rskew  = skewRadius(p, az) + p.altitude_m;
    const double Vhoriz = std::sqrt(static_cast<double>(v_north_mps) * v_north_mps
                                  + static_cast<double>(v_east_mps)  * v_east_mps);
    return Vhoriz / Rskew;
}

Eigen::Vector3d transportRate(const GeodeticPosition& p, float v_north_mps, float v_east_mps)
{
    const double az = std::atan2(v_east_mps, v_north_mps);

    Eigen::Vector3d omega_LL;
    omega_LL(0) =  0.0;
    omega_LL(1) = -horizonRate(p, v_north_mps, v_east_mps);
    omega_LL(2) = -longitudeRate_rad_s(p, v_east_mps);

    return Eigen::AngleAxisd(-az, Eigen::Vector3d::UnitZ()) * omega_LL;
}

// ---------------------------------------------------------------------------
// Gravity and Earth rate
// ---------------------------------------------------------------------------

double gravity_mps2(const GeodeticPosition& p)
{
    // Somigliana model
    const double sin_lat  = std::sin(p.latitude_rad);
    const double sin2_lat = sin_lat * sin_lat;

    const double g0 = 9.7803253359 * (1.0 + 0.00193185265241 * sin2_lat)
                    / std::sqrt(1.0 - kE2 * sin2_lat);

    return g0 - (3.086e-6 - 0.004e-6 * sin2_lat) * p.altitude_m;
}

Eigen::Vector3d omega_ie_n(const GeodeticPosition& p)
{
    Eigen::Matrix3d Cne;
    latLon2Cne(p.latitude_rad, p.longitude_rad, Cne);
    return Cne * Eigen::Vector3d(0.0, 0.0, kOmega_rps);
}

// ---------------------------------------------------------------------------
// ECEF conversions
// ---------------------------------------------------------------------------

Eigen::Vector3d toECEF(const GeodeticPosition& p)
{
    const double N = primeVerticalRadius(p.latitude_rad);
    const double c = std::cos(p.latitude_rad);
    const double s = std::sin(p.latitude_rad);
    Eigen::Vector3d ecef;
    ecef.x() = (N + p.altitude_m) * c * std::cos(p.longitude_rad);
    ecef.y() = (N + p.altitude_m) * c * std::sin(p.longitude_rad);
    ecef.z() = (N * (1.0 - kE2) + p.altitude_m) * s;
    return ecef;
}

GeodeticPosition fromECEF(const Eigen::Vector3d& ecef)
{
    // Newton-Raphson iteration (K-form)
    const double p  = std::sqrt(ecef.x() * ecef.x() + ecef.y() * ecef.y());
    const double K0 = 1.0 / (1.0 - kE2);
    double K        = K0;

    const double oneMinusE2 = 1.0 - kE2;
    const double p2 = p * p;
    const double Z2 = ecef.z() * ecef.z();

    for (int i = 0; i < 5; ++i) {
        const double ci = std::pow(p2 + oneMinusE2 * Z2 * K * K, 1.5) / (kA_m * kE2);
        K = 1.0 + (p2 + oneMinusE2 * Z2 * K * K * K) / (ci - p2);
    }

    GeodeticPosition out;
    out.latitude_rad  = std::atan2(ecef.z() * K, p);
    out.longitude_rad = std::atan2(ecef.y(), ecef.x());
    out.altitude_m    = static_cast<float>(
        (1.0 / kE2) * (1.0 / K - 1.0 / K0) * std::sqrt(p2 + Z2 * K * K));

    normalizeLatLon(&out.latitude_rad, &out.longitude_rad);
    return out;
}

// ---------------------------------------------------------------------------
// NED-to-ECEF rotation
// ---------------------------------------------------------------------------

Eigen::Quaterniond qne(const GeodeticPosition& p)
{
    Eigen::Matrix3d Cne;
    latLon2Cne(p.latitude_rad, p.longitude_rad, Cne);
    return Eigen::Quaterniond(Cne);
}

GeodeticPosition fromQne(const Eigen::Quaterniond& q_ne)
{
    double lat = 0.0, lon = 0.0;
    Cne2LatLon(q_ne.toRotationMatrix(), &lat, &lon);
    return GeodeticPosition{lat, lon, 0.0f};
}

} // namespace liteaero::nav::WGS84
