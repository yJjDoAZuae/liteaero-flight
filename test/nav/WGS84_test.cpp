#define _USE_MATH_DEFINES
#include <liteaero/nav/WGS84.hpp>
#include <gtest/gtest.h>
#include <cmath>

using namespace liteaero::nav;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

TEST(WGS84Test, SemiMajorAxis) {
    EXPECT_DOUBLE_EQ(WGS84::kA_m, 6378137.0);
}

TEST(WGS84Test, InverseFlattening) {
    EXPECT_DOUBLE_EQ(WGS84::kFinv, 298.257223563);
}

// ---------------------------------------------------------------------------
// Radii of curvature
// ---------------------------------------------------------------------------

TEST(WGS84Test, MeridionalRadiusAtEquator) {
    // At equator: M = a*(1-e²) / (1-e²)^(3/2) = a*(1-e²)^(-1/2) ... actually
    // M = a*(1-e²) / (1-e²*0)^(3/2) = a*(1-e²)
    const double finv = WGS84::kFinv;
    const double e2   = 2.0/finv - 1.0/(finv*finv);
    const double expected = WGS84::kA_m * (1.0 - e2);
    EXPECT_NEAR(WGS84::meridionalRadius(0.0), expected, 1.0);  // 1 m tolerance
}

TEST(WGS84Test, MeridionalRadiusAtPole) {
    // At 90°: denominator is (1-e²)^(3/2), numerator is a*(1-e²) → M = a/sqrt(1-e²)
    const double finv = WGS84::kFinv;
    const double e2   = 2.0/finv - 1.0/(finv*finv);
    const double expected = WGS84::kA_m / std::sqrt(1.0 - e2);
    EXPECT_NEAR(WGS84::meridionalRadius(M_PI / 2.0), expected, 1.0);
}

TEST(WGS84Test, NorthRadiusEqualsMeridionalRadius) {
    GeodeticPosition p{0.5, 1.0, 100.0f};
    EXPECT_DOUBLE_EQ(WGS84::northRadius(p), WGS84::meridionalRadius(p.latitude_rad));
}

TEST(WGS84Test, EastRadiusAtEquator) {
    // At equator: N = a, east radius = N*cos(0) = a
    GeodeticPosition p{0.0, 0.0, 0.0f};
    EXPECT_NEAR(WGS84::eastRadius(p), WGS84::kA_m, 10.0);
}

TEST(WGS84Test, EastRadiusAtPole) {
    // At pole: cos(90°) = 0, east radius → 0
    GeodeticPosition p{M_PI / 2.0, 0.0, 0.0f};
    EXPECT_NEAR(WGS84::eastRadius(p), 0.0, 1.0);
}

// ---------------------------------------------------------------------------
// ECEF round-trip
// ---------------------------------------------------------------------------

TEST(WGS84Test, ECEFRoundTripEquator) {
    GeodeticPosition p{0.0, 0.0, 0.0f};
    Eigen::Vector3d ecef = WGS84::toECEF(p);
    // At equator prime meridian: ECEF = [a, 0, 0]
    EXPECT_NEAR(ecef.x(), WGS84::kA_m, 1e-3);
    EXPECT_NEAR(ecef.y(), 0.0,         1e-3);
    EXPECT_NEAR(ecef.z(), 0.0,         1e-3);

    GeodeticPosition p2 = WGS84::fromECEF(ecef);
    EXPECT_NEAR(p2.latitude_rad,  0.0, 1e-9);
    EXPECT_NEAR(p2.longitude_rad, 0.0, 1e-9);
    EXPECT_NEAR(p2.altitude_m,    0.0f, 1e-3f);
}

TEST(WGS84Test, ECEFRoundTripMidLatitude) {
    GeodeticPosition p{0.5, 1.0, 100.0f};
    GeodeticPosition p2 = WGS84::fromECEF(WGS84::toECEF(p));
    EXPECT_NEAR(p2.latitude_rad,  p.latitude_rad,  1e-9);
    EXPECT_NEAR(p2.longitude_rad, p.longitude_rad, 1e-9);
    EXPECT_NEAR(p2.altitude_m,    p.altitude_m,    1e-3f);
}

TEST(WGS84Test, ECEFKnownValues) {
    // Verified against liteaero-sim WGS84_Datum::ECEF() for lat=0.5, lon=1.0, alt=100m
    GeodeticPosition p{0.5, 1.0, 100.0f};
    Eigen::Vector3d ecef = WGS84::toECEF(p);
    EXPECT_NEAR(ecef.x(), 3026633.4955051164, 1e-3);
    EXPECT_NEAR(ecef.y(), 4713702.3855989361, 1e-3);
    EXPECT_NEAR(ecef.z(), 3039758.8494056859, 1e-3);
}

// ---------------------------------------------------------------------------
// qne round-trip
// ---------------------------------------------------------------------------

TEST(WGS84Test, QneRoundTripEquatorPrimeMeridian) {
    GeodeticPosition p{0.0, 0.0, 0.0f};
    Eigen::Quaterniond q = WGS84::qne(p);
    // At lat=0, lon=0: q_ne should rotate North (z-down NED) to ECEF properly
    // w = cos(π/4), y = sin(π/4) (rotation by π/2 about Y)
    EXPECT_NEAR(q.w(), std::cos(M_PI / 4.0), 1e-9);
    EXPECT_NEAR(q.x(), 0.0,                  1e-9);
    EXPECT_NEAR(q.y(), std::sin(M_PI / 4.0), 1e-9);
    EXPECT_NEAR(q.z(), 0.0,                  1e-9);
}

TEST(WGS84Test, QneRoundTripMidLatitude) {
    GeodeticPosition p{0.5, 1.0, 0.0f};
    GeodeticPosition p2 = WGS84::fromQne(WGS84::qne(p));
    EXPECT_NEAR(p2.latitude_rad,  p.latitude_rad,  1e-9);
    EXPECT_NEAR(p2.longitude_rad, p.longitude_rad, 1e-9);
}

// ---------------------------------------------------------------------------
// Gravity
// ---------------------------------------------------------------------------

TEST(WGS84Test, GravityAtEquator) {
    GeodeticPosition p{0.0, 0.0, 0.0f};
    double g = WGS84::gravity_mps2(p);
    EXPECT_NEAR(g, 9.7803, 1e-3);  // standard equatorial gravity
}

TEST(WGS84Test, GravityAtPole) {
    GeodeticPosition p{M_PI / 2.0, 0.0, 0.0f};
    double g = WGS84::gravity_mps2(p);
    EXPECT_GT(g, 9.80);  // polar gravity > equatorial
    EXPECT_LT(g, 9.84);
}

TEST(WGS84Test, GravityDecreasesWithAltitude) {
    GeodeticPosition p_low {0.5, 0.0,       0.0f};
    GeodeticPosition p_high{0.5, 0.0, 10000.0f};
    EXPECT_GT(WGS84::gravity_mps2(p_low), WGS84::gravity_mps2(p_high));
}

// ---------------------------------------------------------------------------
// Kinematic rates
// ---------------------------------------------------------------------------

TEST(WGS84Test, LatitudeRateNorthAtEquator) {
    // lat_rate = v_north / (northRadius + alt) ≈ v_north / kA_m at equator
    GeodeticPosition p{0.0, 0.0, 0.0f};
    const float v_north = 100.0f;
    double rate = WGS84::latitudeRate_rad_s(p, v_north);
    EXPECT_NEAR(rate, static_cast<double>(v_north) / WGS84::northRadius(p), 1e-12);
}

TEST(WGS84Test, LongitudeRateEastAtEquator) {
    GeodeticPosition p{0.0, 0.0, 0.0f};
    const float v_east = 100.0f;
    double rate = WGS84::longitudeRate_rad_s(p, v_east);
    EXPECT_NEAR(rate, static_cast<double>(v_east) / WGS84::eastRadius(p), 1e-12);
}
