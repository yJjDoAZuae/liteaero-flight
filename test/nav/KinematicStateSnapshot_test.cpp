#define _USE_MATH_DEFINES
#include <liteaero/nav/KinematicStateUtil.hpp>
#include <gtest/gtest.h>
#include <cmath>

using namespace liteaero::nav;
namespace KSU = liteaero::nav::KinematicStateUtil;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static KinematicStateSnapshot makeSnapshot(
    Eigen::Quaternionf q_nw           = Eigen::Quaternionf::Identity(),
    float alpha_rad                   = 0.0f,
    float beta_rad                    = 0.0f,
    Eigen::Vector3f velocity_ned_mps  = {50.0f, 0.0f, 0.0f},
    Eigen::Vector3f wind_ned_mps      = Eigen::Vector3f::Zero(),
    Eigen::Vector3f accel_ned_mps2    = Eigen::Vector3f::Zero(),
    Eigen::Vector3f rates_body_rps    = Eigen::Vector3f::Zero())
{
    KinematicStateSnapshot s;
    s.q_nw              = q_nw;
    s.alpha_rad         = alpha_rad;
    s.beta_rad          = beta_rad;
    s.velocity_ned_mps  = velocity_ned_mps;
    s.wind_ned_mps      = wind_ned_mps;
    s.acceleration_ned_mps2 = accel_ned_mps2;
    s.rates_body_rps    = rates_body_rps;
    return s;
}

// ---------------------------------------------------------------------------
// Construction / default state
// ---------------------------------------------------------------------------

TEST(KinematicStateSnapshotTest, DefaultConstruction) {
    KinematicStateSnapshot s;
    EXPECT_DOUBLE_EQ(s.time_s, 0.0);
    EXPECT_FLOAT_EQ(s.alpha_rad,  0.0f);
    EXPECT_FLOAT_EQ(s.beta_rad,   0.0f);
    EXPECT_TRUE(s.q_nw.isApprox(Eigen::Quaternionf::Identity(), 1e-6f));
    EXPECT_TRUE(s.velocity_ned_mps.isZero());
    EXPECT_TRUE(s.wind_ned_mps.isZero());
}

// ---------------------------------------------------------------------------
// q_nb derivation
// ---------------------------------------------------------------------------

TEST(KinematicStateSnapshotTest, QnbEqualsQnwTimesQwbZeroAngles) {
    // alpha=0, beta=0 => q_wb=Identity => q_nb = q_nw
    KinematicStateSnapshot s = makeSnapshot();
    auto qnb = KSU::q_nb(s);
    EXPECT_TRUE(qnb.isApprox(Eigen::Quaternionf::Identity(), 1e-5f));
}

TEST(KinematicStateSnapshotTest, QnbEqualsQnwTimesQwbWithAlpha) {
    const float alpha = 0.2f;
    KinematicStateSnapshot s = makeSnapshot(Eigen::Quaternionf::Identity(), alpha, 0.0f);
    auto qnb = KSU::q_nb(s);
    // q_wb = Ry(alpha), so q_nb = Identity * Ry(alpha) = Ry(alpha)
    Eigen::Quaternionf expected(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitY()));
    EXPECT_TRUE(qnb.isApprox(expected, 1e-5f));
}

TEST(KinematicStateSnapshotTest, QnbEqualsQnwTimesQwbWithBeta) {
    const float beta = 0.1f;
    KinematicStateSnapshot s = makeSnapshot(Eigen::Quaternionf::Identity(), 0.0f, beta);
    auto qnb = KSU::q_nb(s);
    // q_wb = Rz(-beta), so q_nb = Rz(-beta)
    Eigen::Quaternionf expected(Eigen::AngleAxisf(-beta, Eigen::Vector3f::UnitZ()));
    EXPECT_TRUE(qnb.isApprox(expected, 1e-5f));
}

// ---------------------------------------------------------------------------
// Euler angle round-trip
// ---------------------------------------------------------------------------

TEST(KinematicStateSnapshotTest, RollPitchHeadingLevelFlight) {
    KinematicStateSnapshot s = makeSnapshot();
    EXPECT_NEAR(KSU::roll_rad(s),    0.0f, 1e-5f);
    EXPECT_NEAR(KSU::pitch_rad(s),   0.0f, 1e-5f);
    EXPECT_NEAR(KSU::heading_rad(s), 0.0f, 1e-5f);
}

TEST(KinematicStateSnapshotTest, RollAngleCorrect) {
    const float phi = 0.3f;
    // q_nw = Identity, alpha = 0 => q_nb = Identity; then roll q_nb by phi around body X
    // But to test roll_rad we need q_nb with roll: set q_nw = Rx(phi)
    Eigen::Quaternionf q_nw(Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitX()));
    KinematicStateSnapshot s = makeSnapshot(q_nw, 0.0f, 0.0f);
    EXPECT_NEAR(KSU::roll_rad(s), phi, 1e-4f);
}

TEST(KinematicStateSnapshotTest, PitchAngleCorrect) {
    const float theta = 0.2f;
    Eigen::Quaternionf q_nw(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
    KinematicStateSnapshot s = makeSnapshot(q_nw, 0.0f, 0.0f);
    EXPECT_NEAR(KSU::pitch_rad(s), theta, 1e-4f);
}

// ---------------------------------------------------------------------------
// Velocity frame transforms
// ---------------------------------------------------------------------------

TEST(KinematicStateSnapshotTest, VelocityWindZeroWind) {
    // Zero wind, identity q_nw: wind-frame velocity equals NED velocity
    KinematicStateSnapshot s = makeSnapshot();
    auto v_wind = KSU::velocity_wind_mps(s);
    EXPECT_NEAR(v_wind.x(), 50.0f, 1e-4f);
    EXPECT_NEAR(v_wind.y(),  0.0f, 1e-4f);
    EXPECT_NEAR(v_wind.z(),  0.0f, 1e-4f);
}

TEST(KinematicStateSnapshotTest, VelocityWindSubtractsWind) {
    // Headwind from north: wind_ned = [-10, 0, 0], vel_ned = [50, 0, 0]
    // Airmass velocity in NED = [60, 0, 0], wind-frame (identity q_nw) = [60, 0, 0]
    KinematicStateSnapshot s = makeSnapshot(
        Eigen::Quaternionf::Identity(), 0.0f, 0.0f,
        {50.0f, 0.0f, 0.0f}, {-10.0f, 0.0f, 0.0f});
    auto v_wind = KSU::velocity_wind_mps(s);
    EXPECT_NEAR(v_wind.x(), 60.0f, 1e-4f);
    EXPECT_NEAR(v_wind.y(),  0.0f, 1e-4f);
}

TEST(KinematicStateSnapshotTest, AirspeedMagnitude) {
    KinematicStateSnapshot s = makeSnapshot(
        Eigen::Quaternionf::Identity(), 0.0f, 0.0f,
        {30.0f, 40.0f, 0.0f}, Eigen::Vector3f::Zero());
    EXPECT_NEAR(KSU::airspeed_mps(s), 50.0f, 1e-3f);
}

TEST(KinematicStateSnapshotTest, VelocityBodyLevelFlight) {
    // Identity q_nb (alpha=beta=0, q_nw=I): body velocity = NED velocity
    KinematicStateSnapshot s = makeSnapshot();
    auto v_body = KSU::velocity_body_mps(s);
    EXPECT_NEAR(v_body.x(), 50.0f, 1e-4f);
    EXPECT_NEAR(v_body.y(),  0.0f, 1e-4f);
    EXPECT_NEAR(v_body.z(),  0.0f, 1e-4f);
}

TEST(KinematicStateSnapshotTest, AccelerationBodyRotatesFromNED) {
    // q_nw = 90° about Z, alpha=beta=0: q_nb same. Body X maps to NED Y.
    // NED accel = [0, 10, 0]; body accel = C_BN * [0,10,0] = [10, 0, 0] reversed...
    // Actually body X maps to NED Y means C_NB * x_body = y_NED
    // So C_BN * [0,10,0] = [10, 0, 0] (body X direction sees NED Y)
    Eigen::Quaternionf q_nw(Eigen::AngleAxisf(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitZ()));
    KinematicStateSnapshot s = makeSnapshot(q_nw);
    s.acceleration_ned_mps2 = {0.0f, 10.0f, 0.0f};
    auto a_body = KSU::acceleration_body_mps2(s);
    EXPECT_NEAR(a_body.x(), 10.0f, 1e-4f);
    EXPECT_NEAR(a_body.y(),  0.0f, 1e-4f);
}

// ---------------------------------------------------------------------------
// q_ns (stability frame)
// ---------------------------------------------------------------------------

TEST(KinematicStateSnapshotTest, QnsEqualsQnwTimesRyAlpha) {
    const float alpha = 0.15f;
    KinematicStateSnapshot s = makeSnapshot(Eigen::Quaternionf::Identity(), alpha, 0.1f);
    auto q_ns = KSU::q_ns(s);
    Eigen::Quaternionf expected(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitY()));
    EXPECT_TRUE(q_ns.isApprox(expected, 1e-5f));
}

// ---------------------------------------------------------------------------
// Plane of motion / turn circle
// ---------------------------------------------------------------------------

TEST(KinematicStateSnapshotTest, PlaneOfMotionZeroAccel) {
    KinematicStateSnapshot s = makeSnapshot();
    PlaneOfMotion pom = KSU::plane_of_motion(s);
    // Zero acceleration → POM undefined → returns Identity
    EXPECT_TRUE(pom.q_np.isApprox(Eigen::Quaternionf::Identity(), 1e-5f));
}

TEST(KinematicStateSnapshotTest, TurnCircleRadiusFromCentripetal) {
    // v = [50, 0, 0], a_perp = [0, 10, 0] → R = V²/|a_perp| = 2500/10 = 250 m
    KinematicStateSnapshot s = makeSnapshot(
        Eigen::Quaternionf::Identity(), 0.0f, 0.0f,
        {50.0f, 0.0f, 0.0f}, Eigen::Vector3f::Zero(),
        {0.0f, 10.0f, 0.0f});
    TurnCircle tc = KSU::turn_circle(s);
    EXPECT_NEAR(tc.turn_center_delta_ned_m.norm(), 250.0f, 1.0f);
}

// ---------------------------------------------------------------------------
// Euler rates
// ---------------------------------------------------------------------------

TEST(KinematicStateSnapshotTest, EulerRatesZeroBodyRates) {
    KinematicStateSnapshot s = makeSnapshot();
    auto rates = KSU::euler_rates_rad_s(s);
    EXPECT_NEAR(rates.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(rates.y(), 0.0f, 1e-6f);
    EXPECT_NEAR(rates.z(), 0.0f, 1e-6f);
}

TEST(KinematicStateSnapshotTest, BodyRatesToEulerRatesRoundTrip) {
    Eigen::Vector3f euler{0.2f, 0.1f, 0.5f};  // roll, pitch, heading
    Eigen::Vector3f body_rates{0.1f, 0.05f, 0.2f};
    auto euler_rates = KSU::body_rates_to_euler_rates(euler, body_rates);
    auto body_rates2 = KSU::euler_rates_to_body_rates(euler, euler_rates);
    EXPECT_NEAR((body_rates2 - body_rates).norm(), 0.0f, 1e-5f);
}

// ---------------------------------------------------------------------------
// JSON round-trip
// ---------------------------------------------------------------------------

TEST(KinematicStateSnapshotTest, JsonRoundTrip) {
    KinematicStateSnapshot s;
    s.time_s                = 1.5;
    s.position              = {0.5, 1.0, 100.0f};
    s.velocity_ned_mps      = {50.0f, 5.0f, -2.0f};
    s.acceleration_ned_mps2 = {0.1f, 0.2f, 9.8f};
    s.q_nw                  = Eigen::Quaternionf(Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitY()));
    s.rates_body_rps        = {0.01f, 0.02f, 0.03f};
    s.alpha_rad             = 0.05f;
    s.beta_rad              = 0.02f;
    s.alpha_dot_rad_s       = 0.001f;
    s.beta_dot_rad_s        = 0.0005f;
    s.roll_rate_wind_rad_s  = 0.1f;
    s.wind_ned_mps          = {-5.0f, 0.0f, 0.0f};

    auto j  = KSU::serializeJson(s);
    auto s2 = KSU::deserializeJson(j);

    EXPECT_DOUBLE_EQ(s2.time_s, s.time_s);
    EXPECT_NEAR(s2.position.latitude_rad,  s.position.latitude_rad,  1e-12);
    EXPECT_NEAR(s2.position.longitude_rad, s.position.longitude_rad, 1e-12);
    EXPECT_NEAR(s2.position.altitude_m,    s.position.altitude_m,    1e-6f);
    EXPECT_TRUE(s2.velocity_ned_mps.isApprox(s.velocity_ned_mps, 1e-6f));
    EXPECT_TRUE(s2.q_nw.isApprox(s.q_nw, 1e-6f));
    EXPECT_NEAR(s2.alpha_rad,            s.alpha_rad,            1e-6f);
    EXPECT_NEAR(s2.beta_rad,             s.beta_rad,             1e-6f);
    EXPECT_NEAR(s2.roll_rate_wind_rad_s, s.roll_rate_wind_rad_s, 1e-6f);
    EXPECT_TRUE(s2.wind_ned_mps.isApprox(s.wind_ned_mps, 1e-6f));
}

// ---------------------------------------------------------------------------
// Proto round-trip
// ---------------------------------------------------------------------------

TEST(KinematicStateSnapshotTest, ProtoRoundTrip) {
    KinematicStateSnapshot s;
    s.time_s                = 2.0;
    s.position              = {0.3, -0.7, 500.0f};
    s.velocity_ned_mps      = {60.0f, -3.0f, 1.0f};
    s.acceleration_ned_mps2 = {0.5f, -0.1f, 9.81f};
    s.q_nw                  = Eigen::Quaternionf::Identity();
    s.alpha_rad             = 0.08f;
    s.beta_rad              = -0.02f;
    s.wind_ned_mps          = {3.0f, 1.0f, 0.0f};

    auto bytes = KSU::serializeProto(s);
    auto s2    = KSU::deserializeProto(bytes);

    EXPECT_DOUBLE_EQ(s2.time_s, s.time_s);
    EXPECT_NEAR(s2.position.latitude_rad,  s.position.latitude_rad,  1e-12);
    EXPECT_NEAR(s2.position.longitude_rad, s.position.longitude_rad, 1e-12);
    EXPECT_NEAR(s2.position.altitude_m,    s.position.altitude_m,    1e-4f);
    EXPECT_TRUE(s2.velocity_ned_mps.isApprox(s.velocity_ned_mps, 1e-6f));
    EXPECT_NEAR(s2.alpha_rad, s.alpha_rad, 1e-6f);
    EXPECT_NEAR(s2.beta_rad,  s.beta_rad,  1e-6f);
    EXPECT_TRUE(s2.wind_ned_mps.isApprox(s.wind_ned_mps, 1e-6f));
}
