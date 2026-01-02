/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
    USV Sailboat Simulator Implementation

    Physical models:
    - WINGSAIL_ROTATION: Symmetric airfoil with rotation control
    - WINGSAIL_FLAP: Cambered airfoil with trailing edge flap
    - WINGSAIL_FREE: Self-balancing symmetric airfoil

    References:
    - "Forces on Sails" - https://en.wikipedia.org/wiki/Forces_on_sails
    - OpenTransat project - flap control algorithms
    - Sailbuoy project - self-balancing wingsail
*/

#include "SIM_Sailboat_USV.h"
#include <cstring>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace SITL_USV {

// ============================================================================
// Lift/Drag Curves
// ============================================================================

// Angle index: 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180

// ROTATION mode: Symmetric NACA 0012-like airfoil
// Higher lift at low angles, stall around 15-20 degrees
const float SailboatUSV::lift_curve_rotation_[AERO_TABLE_SIZE] = {
    0.00f, 0.60f, 1.10f, 1.20f, 1.00f, 0.80f, 0.60f, 0.45f, 0.25f, 0.00f,
    -0.25f, -0.45f, -0.60f, -0.80f, -1.00f, -1.20f, -1.10f, -0.60f, 0.00f
};

const float SailboatUSV::drag_curve_rotation_[AERO_TABLE_SIZE] = {
    0.01f, 0.02f, 0.05f, 0.15f, 0.35f, 0.55f, 0.75f, 0.90f, 1.00f, 1.05f,
    1.00f, 0.90f, 0.75f, 0.55f, 0.35f, 0.15f, 0.05f, 0.02f, 0.01f
};

// FLAP mode: Cambered airfoil with flap
// Based on NACA 23012 with 25% flap
// Flap increases Cl_max significantly
const float SailboatUSV::lift_curve_flap_[AERO_TABLE_SIZE] = {
    0.30f, 0.80f, 1.30f, 1.50f, 1.35f, 1.10f, 0.85f, 0.60f, 0.35f, 0.10f,
    -0.15f, -0.40f, -0.65f, -0.90f, -1.15f, -1.40f, -1.20f, -0.70f, -0.20f
};

const float SailboatUSV::drag_curve_flap_[AERO_TABLE_SIZE] = {
    0.02f, 0.03f, 0.06f, 0.18f, 0.40f, 0.62f, 0.82f, 0.95f, 1.05f, 1.10f,
    1.05f, 0.95f, 0.82f, 0.62f, 0.40f, 0.18f, 0.06f, 0.03f, 0.02f
};

// ============================================================================
// Utility Functions
// ============================================================================

float beaufort_to_wind_speed(BeaufortScale beaufort) {
    // Returns mean wind speed for each Beaufort level in m/s
    static const float speeds[] = {
        0.25f,   // 0: Calm
        1.0f,    // 1: Light air
        2.45f,   // 2: Light breeze
        4.45f,   // 3: Gentle breeze
        6.7f,    // 4: Moderate breeze
        9.35f,   // 5: Fresh breeze
        12.3f,   // 6: Strong breeze
        15.5f,   // 7: Near gale
        18.95f,  // 8: Gale
        22.6f,   // 9: Strong gale
        26.45f,  // 10: Storm
        30.55f,  // 11: Violent storm
        35.0f    // 12: Hurricane
    };
    int idx = static_cast<int>(beaufort);
    if (idx < 0) idx = 0;
    if (idx > 12) idx = 12;
    return speeds[idx];
}

float sea_state_to_wave_height(SeaState state) {
    // Returns significant wave height for each sea state in meters
    static const float heights[] = {
        0.0f,    // 0: Calm glassy
        0.05f,   // 1: Calm rippled
        0.3f,    // 2: Smooth
        0.875f,  // 3: Slight
        1.875f,  // 4: Moderate
        3.25f,   // 5: Rough
        5.0f,    // 6: Very rough
        7.5f,    // 7: High
        11.5f,   // 8: Very high
        14.0f    // 9: Phenomenal
    };
    int idx = static_cast<int>(state);
    if (idx < 0) idx = 0;
    if (idx > 9) idx = 9;
    return heights[idx];
}

BeaufortScale wind_speed_to_beaufort(float speed_ms) {
    if (speed_ms < 0.5f) return BeaufortScale::CALM;
    if (speed_ms < 1.6f) return BeaufortScale::LIGHT_AIR;
    if (speed_ms < 3.4f) return BeaufortScale::LIGHT_BREEZE;
    if (speed_ms < 5.5f) return BeaufortScale::GENTLE_BREEZE;
    if (speed_ms < 8.0f) return BeaufortScale::MODERATE_BREEZE;
    if (speed_ms < 10.8f) return BeaufortScale::FRESH_BREEZE;
    if (speed_ms < 13.9f) return BeaufortScale::STRONG_BREEZE;
    if (speed_ms < 17.2f) return BeaufortScale::NEAR_GALE;
    if (speed_ms < 20.8f) return BeaufortScale::GALE;
    if (speed_ms < 24.5f) return BeaufortScale::STRONG_GALE;
    if (speed_ms < 28.5f) return BeaufortScale::STORM;
    if (speed_ms < 32.7f) return BeaufortScale::VIOLENT_STORM;
    return BeaufortScale::HURRICANE;
}

SeaState wave_height_to_sea_state(float height_m) {
    if (height_m < 0.0f) return SeaState::CALM_GLASSY;
    if (height_m < 0.1f) return SeaState::CALM_RIPPLED;
    if (height_m < 0.5f) return SeaState::SMOOTH;
    if (height_m < 1.25f) return SeaState::SLIGHT;
    if (height_m < 2.5f) return SeaState::MODERATE;
    if (height_m < 4.0f) return SeaState::ROUGH;
    if (height_m < 6.0f) return SeaState::VERY_ROUGH;
    if (height_m < 9.0f) return SeaState::HIGH;
    if (height_m < 14.0f) return SeaState::VERY_HIGH;
    return SeaState::PHENOMENAL;
}

BeaufortScale EnvironmentParams::get_beaufort() const {
    return wind_speed_to_beaufort(wind_speed_ms);
}

SeaState EnvironmentParams::get_sea_state() const {
    return wave_height_to_sea_state(wave_height_m);
}

// ============================================================================
// SailboatUSV Implementation
// ============================================================================

SailboatUSV::SailboatUSV()
    : rudder_input_(0.0f)
    , wingsail_input_(0.0f)
    , throttle_input_(0.0f)
    , wave_phase_(0.0f)
    , sim_time_ms_(0)
    , motor_enabled_(false)
{
    memset(&state_, 0, sizeof(state_));
    memset(&env_, 0, sizeof(env_));

    // Default wingsail parameters (FLAP mode, similar to OpenTransat)
    wingsail_.type = WingsailType::WINGSAIL_FLAP;
    wingsail_.sail_area_m2 = 1.5f;
    wingsail_.aspect_ratio = 4.0f;
    wingsail_.mass_kg = 5.0f;
    wingsail_.rotation_angle_max_deg = 40.0f;
    wingsail_.rotation_angle_ideal_deg = 25.0f;
    wingsail_.flap_chord_ratio = 0.25f;
    wingsail_.flap_angle_max_deg = 16.0f;
    wingsail_.flap_angle_normal_deg = 10.0f;
    wingsail_.free_balance_point = 0.25f;

    // Default hull parameters (small USV catamaran)
    hull_.length_m = 2.0f;
    hull_.beam_m = 1.0f;
    hull_.draft_m = 0.15f;
    hull_.displacement_kg = 30.0f;
    hull_.wetted_area_m2 = 1.5f;
    hull_.drag_coefficient = 0.05f;
    hull_.lift_coefficient = 0.1f;
    hull_.added_mass_x = 0.05f;
    hull_.added_mass_y = 0.8f;
    hull_.rudder_area_m2 = 0.02f;
    hull_.rudder_max_angle_deg = 35.0f;
    hull_.turning_circle_m = 3.0f;

    // Default environment (light breeze, calm sea)
    env_.wind_speed_ms = 5.0f;
    env_.wind_direction_deg = 0.0f;
    env_.wind_turbulence = 0.1f;
    env_.wind_gust_factor = 1.2f;
    env_.wave_enable = false;
    env_.wave_height_m = 0.3f;
    env_.wave_period_s = 4.0f;
    env_.wave_direction_deg = 0.0f;
    env_.wave_spread_deg = 30.0f;
    env_.tide_speed_ms = 0.0f;
    env_.tide_direction_deg = 0.0f;
}

void SailboatUSV::init(WingsailType type) {
    wingsail_.type = type;
    reset();
}

void SailboatUSV::reset() {
    state_.lat_deg = 0.0;
    state_.lon_deg = 0.0;
    state_.altitude_m = 0.0f;
    state_.velocity_ef = Vector3f();
    state_.velocity_bf = Vector3f();
    state_.velocity_water = Vector3f();
    state_.roll_rad = 0.0f;
    state_.pitch_rad = 0.0f;
    state_.yaw_rad = 0.0f;
    state_.gyro = Vector3f();
    state_.accel_bf = Vector3f();
    state_.wingsail_angle_deg = 0.0f;
    state_.flap_angle_deg = 0.0f;
    state_.normalized_control = 0.0f;
    state_.apparent_wind_speed_ms = 0.0f;
    state_.apparent_wind_dir_deg = 0.0f;
    state_.vmg = 0.0f;
    state_.heel_angle_deg = 0.0f;
    state_.leeway_deg = 0.0f;

    rudder_input_ = 0.0f;
    wingsail_input_ = 0.0f;
    throttle_input_ = 0.0f;
    wave_phase_ = 0.0f;
    sim_time_ms_ = 0;
}

void SailboatUSV::set_wingsail_params(const WingsailParams& params) {
    wingsail_ = params;
}

void SailboatUSV::set_hull_params(const HullParams& params) {
    hull_ = params;
}

void SailboatUSV::set_environment(const EnvironmentParams& env) {
    env_ = env;
}

void SailboatUSV::set_rudder_input(float rudder) {
    rudder_input_ = std::max(-1.0f, std::min(1.0f, rudder));
}

void SailboatUSV::set_wingsail_input(float wingsail) {
    wingsail_input_ = std::max(-1.0f, std::min(1.0f, wingsail));
    state_.normalized_control = wingsail_input_;
}

void SailboatUSV::set_throttle_input(float throttle) {
    throttle_input_ = std::max(0.0f, std::min(1.0f, throttle));
}

void SailboatUSV::set_beaufort(BeaufortScale beaufort) {
    env_.wind_speed_ms = beaufort_to_wind_speed(beaufort);

    // Set typical turbulence for the wind level
    float bf = static_cast<float>(beaufort);
    env_.wind_turbulence = 0.05f + bf * 0.02f;
    env_.wind_gust_factor = 1.0f + bf * 0.05f;
}

void SailboatUSV::set_sea_state(SeaState sea_state) {
    env_.wave_enable = (sea_state != SeaState::CALM_GLASSY);
    env_.wave_height_m = sea_state_to_wave_height(sea_state);

    // Wave period roughly follows sqrt(wave_height) * 3
    env_.wave_period_s = 2.0f + sqrtf(env_.wave_height_m) * 3.0f;
}

void SailboatUSV::set_scenario_upwind() {
    env_.wind_direction_deg = 0.0f;     // Wind from North
    state_.yaw_rad = M_PI / 4.0f;       // Heading NE (45 deg)
    set_beaufort(BeaufortScale::MODERATE_BREEZE);
    set_sea_state(SeaState::SLIGHT);
}

void SailboatUSV::set_scenario_downwind() {
    env_.wind_direction_deg = 0.0f;     // Wind from North
    state_.yaw_rad = M_PI;              // Heading South (180 deg)
    set_beaufort(BeaufortScale::FRESH_BREEZE);
    set_sea_state(SeaState::MODERATE);
}

void SailboatUSV::set_scenario_beam_reach() {
    env_.wind_direction_deg = 0.0f;     // Wind from North
    state_.yaw_rad = M_PI / 2.0f;       // Heading East (90 deg)
    set_beaufort(BeaufortScale::GENTLE_BREEZE);
    set_sea_state(SeaState::SMOOTH);
}

void SailboatUSV::set_scenario_storm() {
    env_.wind_direction_deg = 45.0f;    // Wind from NE
    state_.yaw_rad = M_PI * 1.5f;       // Heading West
    set_beaufort(BeaufortScale::GALE);
    set_sea_state(SeaState::ROUGH);
    env_.tide_speed_ms = 1.5f;
    env_.tide_direction_deg = 90.0f;
}

float SailboatUSV::wrap_180(float angle) const {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float SailboatUSV::wrap_360(float angle) const {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

float SailboatUSV::interpolate_curve(const float* curve, float angle_deg) const {
    // Curve is indexed every 10 degrees from 0 to 180
    angle_deg = fabsf(wrap_180(angle_deg));

    if (angle_deg >= 180.0f) return curve[AERO_TABLE_SIZE - 1];
    if (angle_deg <= 0.0f) return curve[0];

    int idx = static_cast<int>(angle_deg / 10.0f);
    if (idx >= AERO_TABLE_SIZE - 1) return curve[AERO_TABLE_SIZE - 1];

    float frac = (angle_deg - idx * 10.0f) / 10.0f;
    return curve[idx] + frac * (curve[idx + 1] - curve[idx]);
}

float SailboatUSV::get_yaw_rate(float rudder, float speed) const {
    if (fabsf(rudder) < 0.01f || fabsf(speed) < 0.1f) {
        return 0.0f;
    }

    float rudder_angle = rudder * hull_.rudder_max_angle_deg;
    float turn_radius = hull_.turning_circle_m / (2.0f * sinf(fabsf(rudder_angle) * M_PI / 180.0f));

    if (turn_radius < 0.5f) turn_radius = 0.5f;

    // yaw rate = speed / radius (rad/s)
    float yaw_rate = speed / turn_radius;

    if (rudder < 0) yaw_rate = -yaw_rate;

    return yaw_rate;
}

void SailboatUSV::update_apparent_wind() {
    // True wind in earth frame (convert direction FROM to vector TO)
    float wind_dir_rad = (env_.wind_direction_deg + 180.0f) * M_PI / 180.0f;
    Vector3f true_wind_ef(
        env_.wind_speed_ms * cosf(wind_dir_rad),
        env_.wind_speed_ms * sinf(wind_dir_rad),
        0.0f
    );

    // Add turbulence
    if (env_.wind_turbulence > 0.0f) {
        float turb = env_.wind_turbulence * env_.wind_speed_ms;
        // Simple pseudo-random turbulence based on sim time
        float t = sim_time_ms_ * 0.001f;
        true_wind_ef.x += turb * sinf(t * 2.1f + 0.5f);
        true_wind_ef.y += turb * cosf(t * 1.7f + 1.2f);
    }

    // Apparent wind = true wind - boat velocity
    Vector3f apparent_wind_ef = true_wind_ef - state_.velocity_ef;

    // Convert to body frame
    float cos_yaw = cosf(state_.yaw_rad);
    float sin_yaw = sinf(state_.yaw_rad);

    Vector3f apparent_wind_bf(
        apparent_wind_ef.x * cos_yaw + apparent_wind_ef.y * sin_yaw,
        -apparent_wind_ef.x * sin_yaw + apparent_wind_ef.y * cos_yaw,
        0.0f
    );

    state_.apparent_wind_speed_ms = apparent_wind_bf.length_xy();
    state_.apparent_wind_dir_deg = atan2f(apparent_wind_bf.y, apparent_wind_bf.x) * 180.0f / M_PI;
}

void SailboatUSV::calc_rotation_wingsail(float apparent_wind_speed, float apparent_wind_angle,
                                          float wingsail_angle, float& lift, float& drag) {
    // Angle of attack = wind angle - wingsail angle
    float aoa = apparent_wind_angle - wingsail_angle;
    aoa = wrap_180(aoa);

    // Look up lift and drag from curves
    float cl = interpolate_curve(lift_curve_rotation_, aoa);
    float cd = interpolate_curve(drag_curve_rotation_, aoa);

    // Force = 0.5 * rho * V^2 * S * C
    // Using simplified coefficient (rho = 1.225 kg/m^3)
    float q = 0.5f * 1.225f * apparent_wind_speed * apparent_wind_speed;
    lift = q * wingsail_.sail_area_m2 * cl;
    drag = q * wingsail_.sail_area_m2 * cd;

    // Sign convention: positive lift is perpendicular to wind, toward port
    if (aoa < 0) lift = -lift;
}

void SailboatUSV::calc_flap_wingsail(float apparent_wind_speed, float apparent_wind_angle,
                                      float flap_angle, float& lift, float& drag) {
    // Flap deflection changes the effective camber
    // Flap angle increases Cl, but also Cd
    float flap_cl_boost = flap_angle / wingsail_.flap_angle_max_deg * 0.5f;
    float flap_cd_boost = fabsf(flap_angle) / wingsail_.flap_angle_max_deg * 0.3f;

    // Self-aligning wingsail - it weathervanes into the wind
    // The wingsail angle is determined by wind direction
    float wingsail_angle = apparent_wind_angle;
    if (apparent_wind_angle > 0) {
        wingsail_angle -= 5.0f;  // Slight offset for lift
    } else {
        wingsail_angle += 5.0f;
    }

    float aoa = apparent_wind_angle - wingsail_angle;
    aoa = wrap_180(aoa);

    float cl = interpolate_curve(lift_curve_flap_, aoa) + flap_cl_boost;
    float cd = interpolate_curve(drag_curve_flap_, aoa) + flap_cd_boost;

    float q = 0.5f * 1.225f * apparent_wind_speed * apparent_wind_speed;
    lift = q * wingsail_.sail_area_m2 * cl;
    drag = q * wingsail_.sail_area_m2 * cd;

    // Flap controls the side the lift goes to
    if (flap_angle < 0) lift = -lift;

    state_.wingsail_angle_deg = wingsail_angle;
}

void SailboatUSV::calc_free_wingsail(float apparent_wind_speed, float apparent_wind_angle,
                                      float& lift, float& drag, float& equilibrium_angle) {
    // Self-balancing wingsail finds its own equilibrium
    // The wingsail weathervanes and produces minimal lift/drag

    // Equilibrium angle is approximately aligned with wind
    equilibrium_angle = apparent_wind_angle;

    // Very small lift (just from asymmetry/imperfections)
    float aoa = 2.0f;  // Small residual AOA
    float cl = 0.1f;   // Minimal lift coefficient
    float cd = 0.02f;  // Low drag (streamlined)

    float q = 0.5f * 1.225f * apparent_wind_speed * apparent_wind_speed;
    lift = q * wingsail_.sail_area_m2 * cl;
    drag = q * wingsail_.sail_area_m2 * cd;

    // Side is random based on small perturbations
    if (apparent_wind_angle > 0) lift = -lift;

    state_.wingsail_angle_deg = equilibrium_angle;
}

void SailboatUSV::update_wingsail_forces(float& lift, float& drag, float& moment) {
    float app_speed = state_.apparent_wind_speed_ms;
    float app_angle = state_.apparent_wind_dir_deg;

    switch (wingsail_.type) {
        case WingsailType::WINGSAIL_ROTATION: {
            // Control input directly sets wingsail angle
            float wingsail_angle = wingsail_input_ * wingsail_.rotation_angle_max_deg;
            state_.wingsail_angle_deg = wingsail_angle;
            calc_rotation_wingsail(app_speed, app_angle, wingsail_angle, lift, drag);
            break;
        }

        case WingsailType::WINGSAIL_FLAP: {
            // Control input sets flap angle
            float flap_angle = wingsail_input_ * wingsail_.flap_angle_max_deg;
            state_.flap_angle_deg = flap_angle;
            calc_flap_wingsail(app_speed, app_angle, flap_angle, lift, drag);
            break;
        }

        case WingsailType::WINGSAIL_FREE: {
            float eq_angle;
            calc_free_wingsail(app_speed, app_angle, lift, drag, eq_angle);
            state_.wingsail_angle_deg = eq_angle;
            state_.flap_angle_deg = 0.0f;
            break;
        }
    }

    // Moment from sail (causes heel)
    // Simplified: moment proportional to lift and sail height
    float sail_height = sqrtf(wingsail_.sail_area_m2 * wingsail_.aspect_ratio);
    moment = lift * sail_height * 0.5f;
}

void SailboatUSV::update_hull_forces(float& drag, float& side_force) {
    float speed = state_.velocity_water.length_xy();

    // Hull drag: D = 0.5 * rho * V^2 * S * Cd
    // Water density = 1025 kg/m^3
    float q = 0.5f * 1025.0f * speed * speed;
    drag = q * hull_.wetted_area_m2 * hull_.drag_coefficient;

    // Side force (leeway resistance)
    // Proportional to sideways velocity component
    float leeway_vel = state_.velocity_bf.y;
    side_force = q * hull_.wetted_area_m2 * hull_.lift_coefficient * (leeway_vel > 0 ? 1.0f : -1.0f);

    // Calculate leeway angle
    if (speed > 0.1f) {
        state_.leeway_deg = atan2f(fabsf(leeway_vel), fabsf(state_.velocity_bf.x)) * 180.0f / M_PI;
    } else {
        state_.leeway_deg = 0.0f;
    }
}

void SailboatUSV::update_rudder_forces(float& force, float& moment) {
    float speed = state_.velocity_water.length_xy();
    float rudder_angle = rudder_input_ * hull_.rudder_max_angle_deg;

    // Rudder force: F = 0.5 * rho * V^2 * S * Cl(angle)
    float q = 0.5f * 1025.0f * speed * speed;

    // Simple rudder lift model: Cl = 2*pi*sin(alpha) for small angles
    float alpha_rad = rudder_angle * M_PI / 180.0f;
    float cl = 2.0f * M_PI * sinf(alpha_rad);
    cl = std::max(-2.0f, std::min(2.0f, cl));  // Clamp

    force = q * hull_.rudder_area_m2 * cl;

    // Moment arm is roughly 40% of hull length from center
    float arm = hull_.length_m * 0.4f;
    moment = force * arm;
}

void SailboatUSV::update_wave_effects(float dt) {
    if (!env_.wave_enable || env_.wave_height_m < 0.01f) {
        return;
    }

    // Update wave phase
    float wave_speed = sqrtf(9.81f * env_.wave_period_s / (2.0f * M_PI));
    float wavelength = wave_speed * env_.wave_period_s;

    // Boat movement relative to waves
    float wave_dir_rad = env_.wave_direction_deg * M_PI / 180.0f;
    float boat_wave_speed = state_.velocity_ef.x * cosf(wave_dir_rad) +
                            state_.velocity_ef.y * sinf(wave_dir_rad);

    float relative_speed = wave_speed - boat_wave_speed;
    wave_phase_ += (relative_speed / wavelength) * 2.0f * M_PI * dt;
    if (wave_phase_ > 2.0f * M_PI) wave_phase_ -= 2.0f * M_PI;

    // Wave slope
    float wave_slope = (env_.wave_height_m / 2.0f) * (2.0f * M_PI / wavelength) * cosf(wave_phase_);

    // Convert wave angle to body frame
    float heading_diff = env_.wave_direction_deg * M_PI / 180.0f - state_.yaw_rad;
    float wave_roll = sinf(heading_diff) * atanf(wave_slope);
    float wave_pitch = cosf(heading_diff) * atanf(wave_slope);

    // Apply as angular rates toward wave equilibrium
    float wave_gain = 2.0f;
    state_.gyro.x += (wave_roll - state_.roll_rad) * wave_gain;
    state_.gyro.y += (wave_pitch - state_.pitch_rad) * wave_gain;

    // Heave (vertical motion)
    float heave_accel = -9.81f * (env_.wave_height_m / 2.0f) * sinf(wave_phase_) / env_.wave_period_s;
    state_.accel_bf.z += heave_accel;
}

void SailboatUSV::update_tide_effects() {
    if (env_.tide_speed_ms < 0.01f) {
        return;
    }

    // Tide as constant velocity offset in earth frame
    float tide_dir_rad = env_.tide_direction_deg * M_PI / 180.0f;
    Vector3f tide_vel(
        env_.tide_speed_ms * cosf(tide_dir_rad),
        env_.tide_speed_ms * sinf(tide_dir_rad),
        0.0f
    );

    // Velocity relative to water = velocity relative to ground - tide
    state_.velocity_water = state_.velocity_ef - tide_vel;
}

void SailboatUSV::update(float dt_s) {
    sim_time_ms_ += static_cast<uint32_t>(dt_s * 1000.0f);

    // Update apparent wind
    update_apparent_wind();

    // Update tide effects (velocity_water)
    update_tide_effects();

    // Calculate forces
    float sail_lift, sail_drag, sail_moment;
    update_wingsail_forces(sail_lift, sail_drag, sail_moment);

    float hull_drag, hull_side_force;
    update_hull_forces(hull_drag, hull_side_force);

    float rudder_force, rudder_moment;
    update_rudder_forces(rudder_force, rudder_moment);

    // Convert sail forces from wind frame to body frame
    float app_wind_rad = state_.apparent_wind_dir_deg * M_PI / 180.0f;
    float sail_force_x = -sail_drag * cosf(app_wind_rad) + sail_lift * sinf(app_wind_rad);
    float sail_force_y = -sail_drag * sinf(app_wind_rad) - sail_lift * cosf(app_wind_rad);

    // Motor force (if equipped)
    float motor_force = 0.0f;
    if (motor_enabled_) {
        // Simple motor model: max force at zero speed, decreasing with speed
        float max_motor_force = hull_.displacement_kg * 0.5f;  // ~0.5 m/s^2 max accel
        float speed_factor = 1.0f - std::min(1.0f, state_.velocity_bf.x / 3.0f);
        motor_force = throttle_input_ * max_motor_force * speed_factor;
    }

    // Total forces in body frame
    float total_force_x = sail_force_x + motor_force - hull_drag;
    float total_force_y = sail_force_y - hull_side_force + rudder_force;

    // Accelerations (F = ma)
    float mass_x = hull_.displacement_kg * (1.0f + hull_.added_mass_x);
    float mass_y = hull_.displacement_kg * (1.0f + hull_.added_mass_y);

    state_.accel_bf.x = total_force_x / mass_x;
    state_.accel_bf.y = total_force_y / mass_y;

    // Yaw dynamics
    float yaw_moment = rudder_moment;  // + sail moment contribution
    float yaw_inertia = hull_.displacement_kg * hull_.length_m * hull_.length_m / 12.0f;
    float yaw_accel = yaw_moment / yaw_inertia;

    // Also add direct yaw rate from steering at speed
    float direct_yaw_rate = get_yaw_rate(rudder_input_, state_.velocity_bf.x);
    state_.gyro.z = state_.gyro.z * 0.9f + direct_yaw_rate * 0.1f + yaw_accel * dt_s;

    // Heel from sail moment
    float heel_moment = sail_moment - hull_.displacement_kg * 9.81f * sinf(state_.roll_rad) * hull_.beam_m * 0.3f;
    float heel_inertia = hull_.displacement_kg * hull_.beam_m * hull_.beam_m / 12.0f;
    state_.gyro.x += (heel_moment / heel_inertia) * dt_s * 0.1f;  // Damped

    // Wave effects
    update_wave_effects(dt_s);

    // Integrate angular rates
    state_.roll_rad += state_.gyro.x * dt_s;
    state_.pitch_rad += state_.gyro.y * dt_s;
    state_.yaw_rad += state_.gyro.z * dt_s;

    // Damp angles (stability)
    state_.roll_rad *= 0.98f;
    state_.pitch_rad *= 0.98f;
    state_.gyro.x *= 0.95f;
    state_.gyro.y *= 0.95f;
    state_.gyro.z *= 0.98f;

    // Wrap yaw
    while (state_.yaw_rad > M_PI) state_.yaw_rad -= 2.0f * M_PI;
    while (state_.yaw_rad < -M_PI) state_.yaw_rad += 2.0f * M_PI;

    // Update body frame velocity
    state_.velocity_bf.x += state_.accel_bf.x * dt_s;
    state_.velocity_bf.y += state_.accel_bf.y * dt_s;

    // Speed limits (friction model)
    state_.velocity_bf.x *= 0.995f;
    state_.velocity_bf.y *= 0.95f;  // More damping on sideways motion

    // Convert to earth frame
    float cos_yaw = cosf(state_.yaw_rad);
    float sin_yaw = sinf(state_.yaw_rad);
    state_.velocity_ef.x = state_.velocity_bf.x * cos_yaw - state_.velocity_bf.y * sin_yaw;
    state_.velocity_ef.y = state_.velocity_bf.x * sin_yaw + state_.velocity_bf.y * cos_yaw;

    // Add tide to get ground-relative velocity
    if (env_.tide_speed_ms > 0.01f) {
        float tide_dir_rad = env_.tide_direction_deg * M_PI / 180.0f;
        state_.velocity_ef.x += env_.tide_speed_ms * cosf(tide_dir_rad);
        state_.velocity_ef.y += env_.tide_speed_ms * sinf(tide_dir_rad);
    }

    // Update position (simple integration, no lat/lon conversion)
    // Position in meters from origin
    state_.lat_deg += state_.velocity_ef.x * dt_s / 111320.0;  // approx m per degree
    state_.lon_deg += state_.velocity_ef.y * dt_s / (111320.0 * cosf(static_cast<float>(state_.lat_deg) * M_PI / 180.0f));

    // Calculate VMG (Velocity Made Good toward wind)
    float target_dir_rad = env_.wind_direction_deg * M_PI / 180.0f;
    float boat_speed = state_.velocity_ef.length_xy();
    float boat_dir_rad = atan2f(state_.velocity_ef.y, state_.velocity_ef.x);
    state_.vmg = boat_speed * cosf(boat_dir_rad - target_dir_rad);

    // Update heel angle for display
    state_.heel_angle_deg = state_.roll_rad * 180.0f / M_PI;
}

} // namespace SITL_USV
