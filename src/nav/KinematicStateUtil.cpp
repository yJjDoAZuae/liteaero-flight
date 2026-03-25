#define _USE_MATH_DEFINES
#include <liteaero/nav/KinematicStateUtil.hpp>
#include <liteaero/nav/WGS84.hpp>
#include <liteaero_flight.pb.h>
#include <cmath>
#include <stdexcept>

// ---------------------------------------------------------------------------
// File-scope serialization helpers
// ---------------------------------------------------------------------------

namespace {

static nlohmann::json vec3ToJson(const Eigen::Vector3f& v) {
    return {{"x", v.x()}, {"y", v.y()}, {"z", v.z()}};
}

static nlohmann::json quatToJson(const Eigen::Quaternionf& q) {
    return {{"w", q.w()}, {"x", q.x()}, {"y", q.y()}, {"z", q.z()}};
}

static Eigen::Vector3f vec3FromJson(const nlohmann::json& j) {
    return {j.at("x").get<float>(), j.at("y").get<float>(), j.at("z").get<float>()};
}

static Eigen::Quaternionf quatFromJson(const nlohmann::json& j) {
    return Eigen::Quaternionf(j.at("w").get<float>(), j.at("x").get<float>(),
                              j.at("y").get<float>(), j.at("z").get<float>());
}

static void fillVec3(liteaeroflight::Vector3f* msg, const Eigen::Vector3f& v) {
    msg->set_x(v.x()); msg->set_y(v.y()); msg->set_z(v.z());
}

static void fillQuat(liteaeroflight::Quaternionf* msg, const Eigen::Quaternionf& q) {
    msg->set_w(q.w()); msg->set_x(q.x()); msg->set_y(q.y()); msg->set_z(q.z());
}

} // namespace

namespace liteaero::nav::KinematicStateUtil {

// ---------------------------------------------------------------------------
// Orientation
// ---------------------------------------------------------------------------

Eigen::Quaternionf q_nb(const KinematicStateSnapshot& s)
{
    return (s.q_nw
            * Eigen::AngleAxisf( s.alpha_rad, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(-s.beta_rad,  Eigen::Vector3f::UnitZ())).normalized();
}

Eigen::Quaternionf q_ns(const KinematicStateSnapshot& s)
{
    return (s.q_nw
            * Eigen::AngleAxisf(s.alpha_rad, Eigen::Vector3f::UnitY())).normalized();
}

Eigen::Quaternionf q_nl(const KinematicStateSnapshot& s)
{
    return Eigen::Quaternionf(WGS84::qne(s.position).cast<float>());
}

// ---------------------------------------------------------------------------
// Euler angles
// ---------------------------------------------------------------------------

Eigen::Vector3f roll_pitch_heading(const KinematicStateSnapshot& s)
{
    // ZYX decomposition of q_nb: eulerAngles(2,1,0) returns [yaw, pitch, roll]
    const Eigen::Vector3f ypr = q_nb(s).toRotationMatrix().eulerAngles(2, 1, 0);
    return {ypr(2), ypr(1), ypr(0)};  // [roll, pitch, heading]
}

float roll_rad(const KinematicStateSnapshot& s)
{
    return roll_pitch_heading(s)(0);
}

float pitch_rad(const KinematicStateSnapshot& s)
{
    return roll_pitch_heading(s)(1);
}

float heading_rad(const KinematicStateSnapshot& s)
{
    return roll_pitch_heading(s)(2);
}

Eigen::Vector3f euler_rates_rad_s(const KinematicStateSnapshot& s)
{
    const Eigen::Vector3f euler = roll_pitch_heading(s);
    return body_rates_to_euler_rates(euler, s.rates_body_rps);
}

// ---------------------------------------------------------------------------
// Frame-transformed velocities
// ---------------------------------------------------------------------------

Eigen::Vector3f velocity_body_mps(const KinematicStateSnapshot& s)
{
    return q_nb(s).toRotationMatrix().transpose() * s.velocity_ned_mps;
}

Eigen::Vector3f velocity_stab_mps(const KinematicStateSnapshot& s)
{
    return q_ns(s).toRotationMatrix().transpose() * s.velocity_ned_mps;
}

Eigen::Vector3f velocity_wind_mps(const KinematicStateSnapshot& s)
{
    return s.q_nw.toRotationMatrix().transpose() * (s.velocity_ned_mps - s.wind_ned_mps);
}

float airspeed_mps(const KinematicStateSnapshot& s)
{
    return (s.velocity_ned_mps - s.wind_ned_mps).norm();
}

// ---------------------------------------------------------------------------
// Frame-transformed accelerations
// ---------------------------------------------------------------------------

Eigen::Vector3f acceleration_body_mps2(const KinematicStateSnapshot& s)
{
    return q_nb(s).toRotationMatrix().transpose() * s.acceleration_ned_mps2;
}

Eigen::Vector3f acceleration_wind_mps2(const KinematicStateSnapshot& s)
{
    return s.q_nw.toRotationMatrix().transpose() * s.acceleration_ned_mps2;
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

float crab_rad(const KinematicStateSnapshot& s)
{
    const Eigen::Vector3f v_aero = s.velocity_ned_mps - s.wind_ned_mps;
    const float az_vel  = std::atan2(s.velocity_ned_mps(1), s.velocity_ned_mps(0));
    const float az_wind = std::atan2(v_aero(1), v_aero(0));

    float crab = az_wind - az_vel;
    // wrap to [-π, π]
    crab = std::fmod(crab + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI));
    if (crab < 0.0f) { crab += 2.0f * static_cast<float>(M_PI); }
    return crab - static_cast<float>(M_PI);
}

float crab_rate_rad_s(const KinematicStateSnapshot& s)
{
    const Eigen::Vector3f& v     = s.velocity_ned_mps;
    const Eigen::Vector3f& a     = s.acceleration_ned_mps2;
    const Eigen::Vector3f  v_aero = v - s.wind_ned_mps;

    // d/dt atan2(y,x) = (x*ydot - y*xdot) / (x^2 + y^2)
    const float denom_vel  = v(0)*v(0)     + v(1)*v(1);
    const float denom_aero = v_aero(0)*v_aero(0) + v_aero(1)*v_aero(1);

    if (denom_vel < 1e-6f || denom_aero < 1e-6f) { return 0.0f; }

    const float az_vel_dot  = (v(0)*a(1)     - v(1)*a(0))     / denom_vel;
    const float az_aero_dot = (v_aero(0)*a(1) - v_aero(1)*a(0)) / denom_aero;

    return az_aero_dot - az_vel_dot;
}

PlaneOfMotion plane_of_motion(const KinematicStateSnapshot& s)
{
    static constexpr float kSmallV     = 0.1f;
    static constexpr float kSmallAperp = 1e-4f;

    PlaneOfMotion pom;
    const Eigen::Vector3f v = s.velocity_ned_mps - s.wind_ned_mps;
    const float V = v.norm();

    if (V < kSmallV) { return pom; }

    const Eigen::Vector3f xhat   = v / V;
    const Eigen::Vector3f a_par  = s.acceleration_ned_mps2.dot(xhat) * xhat;
    const Eigen::Vector3f a_perp = s.acceleration_ned_mps2 - a_par;

    if (a_perp.norm() < kSmallAperp) { return pom; }

    const Eigen::Vector3f yhat = a_perp.normalized();
    const Eigen::Vector3f zhat = xhat.cross(yhat);

    Eigen::Matrix3f C_NP;
    C_NP.col(0) = xhat;
    C_NP.col(1) = yhat;
    C_NP.col(2) = zhat;
    pom.q_np = Eigen::Quaternionf(C_NP);
    return pom;
}

TurnCircle turn_circle(const KinematicStateSnapshot& s)
{
    static constexpr float kSmallV     = 0.1f;
    static constexpr float kSmallAperp = 1e-4f;

    TurnCircle tc;
    tc.pom = plane_of_motion(s);

    const Eigen::Vector3f v = s.velocity_ned_mps - s.wind_ned_mps;
    const float V = v.norm();

    if (V < kSmallV) { return tc; }

    const Eigen::Vector3f xhat   = v / V;
    const Eigen::Vector3f a_par  = s.acceleration_ned_mps2.dot(xhat) * xhat;
    const Eigen::Vector3f a_perp = s.acceleration_ned_mps2 - a_par;

    if (a_perp.norm() < kSmallAperp) { return tc; }

    const float R = (V * V) / a_perp.norm();
    tc.turn_center_delta_ned_m = R * a_perp.normalized();
    return tc;
}

// ---------------------------------------------------------------------------
// Euler rate / body rate conversions
// ---------------------------------------------------------------------------

Eigen::Vector3f euler_rates_to_body_rates(const Eigen::Vector3f& euler_rad,
                                           const Eigen::Vector3f& euler_rates_rps)
{
    const float sphi = std::sin(euler_rad(0));
    const float cphi = std::cos(euler_rad(0));
    const float stht = std::sin(euler_rad(1));
    const float ctht = std::cos(euler_rad(1));

    Eigen::Matrix3f C;
    C << 1, 0, -stht, 0, cphi, sphi*ctht, 0, -sphi, cphi*ctht;
    return C * euler_rates_rps;
}

Eigen::Vector3f body_rates_to_euler_rates(const Eigen::Vector3f& euler_rad,
                                           const Eigen::Vector3f& body_rates_rps)
{
    static constexpr float kGimbalTol = 1e-6f;

    const float sphi = std::sin(euler_rad(0));
    const float cphi = std::cos(euler_rad(0));
    const float stht = std::sin(euler_rad(1));
    const float ctht = std::cos(euler_rad(1));

    if (std::fabs(ctht) < kGimbalTol) {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Matrix3f C;
    C << 1, sphi*stht/ctht, cphi*stht/ctht,
         0, cphi,          -sphi,
         0, sphi/ctht,      cphi/ctht;
    return C * body_rates_rps;
}

// ---------------------------------------------------------------------------
// JSON serialization
// ---------------------------------------------------------------------------

nlohmann::json serializeJson(const KinematicStateSnapshot& s)
{
    return {
        {"schema_version",        1},
        {"type",                  "KinematicStateSnapshot"},
        {"time_s",                s.time_s},
        {"position", {
            {"latitude_rad",  s.position.latitude_rad},
            {"longitude_rad", s.position.longitude_rad},
            {"altitude_m",    s.position.altitude_m}
        }},
        {"velocity_ned_mps",      vec3ToJson(s.velocity_ned_mps)},
        {"acceleration_ned_mps2", vec3ToJson(s.acceleration_ned_mps2)},
        {"q_nw",                  quatToJson(s.q_nw)},
        {"rates_body_rps",        vec3ToJson(s.rates_body_rps)},
        {"alpha_rad",             s.alpha_rad},
        {"beta_rad",              s.beta_rad},
        {"alpha_dot_rad_s",       s.alpha_dot_rad_s},
        {"beta_dot_rad_s",        s.beta_dot_rad_s},
        {"roll_rate_wind_rad_s",  s.roll_rate_wind_rad_s},
        {"wind_ned_mps",          vec3ToJson(s.wind_ned_mps)}
    };
}

KinematicStateSnapshot deserializeJson(const nlohmann::json& j)
{
    if (j.at("schema_version").get<int>() != 1)
        throw std::runtime_error("KinematicStateSnapshot::deserializeJson: unsupported schema_version");

    KinematicStateSnapshot s;
    s.time_s = j.at("time_s").get<double>();

    const auto& pos = j.at("position");
    s.position.latitude_rad  = pos.at("latitude_rad").get<double>();
    s.position.longitude_rad = pos.at("longitude_rad").get<double>();
    s.position.altitude_m    = pos.at("altitude_m").get<float>();

    s.velocity_ned_mps      = vec3FromJson(j.at("velocity_ned_mps"));
    s.acceleration_ned_mps2 = vec3FromJson(j.at("acceleration_ned_mps2"));
    s.q_nw                  = quatFromJson(j.at("q_nw"));
    s.rates_body_rps        = vec3FromJson(j.at("rates_body_rps"));
    s.alpha_rad             = j.at("alpha_rad").get<float>();
    s.beta_rad              = j.at("beta_rad").get<float>();
    s.alpha_dot_rad_s       = j.at("alpha_dot_rad_s").get<float>();
    s.beta_dot_rad_s        = j.at("beta_dot_rad_s").get<float>();
    s.roll_rate_wind_rad_s  = j.at("roll_rate_wind_rad_s").get<float>();
    s.wind_ned_mps          = vec3FromJson(j.at("wind_ned_mps"));
    return s;
}

// ---------------------------------------------------------------------------
// Proto serialization
// ---------------------------------------------------------------------------

std::vector<uint8_t> serializeProto(const KinematicStateSnapshot& s)
{
    liteaeroflight::KinematicStateSnapshot msg;
    msg.set_schema_version(1);
    msg.set_time_s(s.time_s);

    auto* pos = msg.mutable_position();
    pos->set_latitude_rad(s.position.latitude_rad);
    pos->set_longitude_rad(s.position.longitude_rad);
    pos->set_altitude_m(s.position.altitude_m);

    fillVec3(msg.mutable_velocity_ned_mps(),      s.velocity_ned_mps);
    fillVec3(msg.mutable_acceleration_ned_mps2(), s.acceleration_ned_mps2);
    fillQuat(msg.mutable_q_nw(),                  s.q_nw);
    fillVec3(msg.mutable_rates_body_rps(),         s.rates_body_rps);

    msg.set_alpha_rad(s.alpha_rad);
    msg.set_beta_rad(s.beta_rad);
    msg.set_alpha_dot_rad_s(s.alpha_dot_rad_s);
    msg.set_beta_dot_rad_s(s.beta_dot_rad_s);
    msg.set_roll_rate_wind_rad_s(s.roll_rate_wind_rad_s);

    fillVec3(msg.mutable_wind_ned_mps(), s.wind_ned_mps);

    std::string buf;
    msg.SerializeToString(&buf);
    return {buf.begin(), buf.end()};
}

KinematicStateSnapshot deserializeProto(const std::vector<uint8_t>& bytes)
{
    liteaeroflight::KinematicStateSnapshot msg;
    if (!msg.ParseFromArray(bytes.data(), static_cast<int>(bytes.size())))
        throw std::runtime_error("KinematicStateSnapshot::deserializeProto: parse failed");
    if (msg.schema_version() != 1)
        throw std::runtime_error("KinematicStateSnapshot::deserializeProto: unsupported schema_version");

    KinematicStateSnapshot s;
    s.time_s = msg.time_s();

    const auto& pos = msg.position();
    s.position.latitude_rad  = pos.latitude_rad();
    s.position.longitude_rad = pos.longitude_rad();
    s.position.altitude_m    = pos.altitude_m();

    const auto& v = msg.velocity_ned_mps();
    const auto& a = msg.acceleration_ned_mps2();
    const auto& q = msg.q_nw();
    const auto& r = msg.rates_body_rps();
    const auto& w = msg.wind_ned_mps();

    s.velocity_ned_mps      = {v.x(), v.y(), v.z()};
    s.acceleration_ned_mps2 = {a.x(), a.y(), a.z()};
    s.q_nw                  = Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());
    s.rates_body_rps        = {r.x(), r.y(), r.z()};
    s.alpha_rad             = msg.alpha_rad();
    s.beta_rad              = msg.beta_rad();
    s.alpha_dot_rad_s       = msg.alpha_dot_rad_s();
    s.beta_dot_rad_s        = msg.beta_dot_rad_s();
    s.roll_rate_wind_rad_s  = msg.roll_rate_wind_rad_s();
    s.wind_ned_mps          = {w.x(), w.y(), w.z()};
    return s;
}

} // namespace liteaero::nav::KinematicStateUtil
