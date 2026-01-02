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
    USV Sailboat Simulator - Extended version for USV project

    Supports three wingsail control types:
    - WINGSAIL_ROTATION: Rudder + wingsail rotation (controls entire wingsail angle)
    - WINGSAIL_FLAP: Rudder + flap (controls flap on wingsail, OpenTransat style)
    - WINGSAIL_FREE: Rudder only (self-balancing wingsail, Sailbuoy style)

    Additional features:
    - Beaufort wind scale support
    - Sea state classification (Douglas scale)
    - Enhanced wave/tide simulation
    - Multi-hull dynamics option
*/

#pragma once

#include <cstdint>
#include <cmath>

namespace SITL_USV {

// Wingsail type enumeration (matches Rover/sailboat.h)
enum class WingsailType : uint8_t {
    WINGSAIL_ROTATION = 0,  // Rudder + wingsail rotation
    WINGSAIL_FLAP = 1,      // Rudder + flap control
    WINGSAIL_FREE = 2       // Rudder only, self-balancing wingsail
};

// Beaufort wind scale
enum class BeaufortScale : uint8_t {
    CALM = 0,           // < 0.5 m/s
    LIGHT_AIR = 1,      // 0.5 - 1.5 m/s
    LIGHT_BREEZE = 2,   // 1.6 - 3.3 m/s
    GENTLE_BREEZE = 3,  // 3.4 - 5.5 m/s
    MODERATE_BREEZE = 4,// 5.5 - 7.9 m/s
    FRESH_BREEZE = 5,   // 8.0 - 10.7 m/s
    STRONG_BREEZE = 6,  // 10.8 - 13.8 m/s
    NEAR_GALE = 7,      // 13.9 - 17.1 m/s
    GALE = 8,           // 17.2 - 20.7 m/s
    STRONG_GALE = 9,    // 20.8 - 24.4 m/s
    STORM = 10,         // 24.5 - 28.4 m/s
    VIOLENT_STORM = 11, // 28.5 - 32.6 m/s
    HURRICANE = 12      // >= 32.7 m/s
};

// Douglas sea state scale
enum class SeaState : uint8_t {
    CALM_GLASSY = 0,    // Wave height 0 m
    CALM_RIPPLED = 1,   // 0 - 0.1 m
    SMOOTH = 2,         // 0.1 - 0.5 m
    SLIGHT = 3,         // 0.5 - 1.25 m
    MODERATE = 4,       // 1.25 - 2.5 m
    ROUGH = 5,          // 2.5 - 4 m
    VERY_ROUGH = 6,     // 4 - 6 m
    HIGH = 7,           // 6 - 9 m
    VERY_HIGH = 8,      // 9 - 14 m
    PHENOMENAL = 9      // >= 14 m
};

// 3D Vector class for simulation
struct Vector3f {
    float x, y, z;

    Vector3f() : x(0), y(0), z(0) {}
    Vector3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    Vector3f operator+(const Vector3f& v) const { return Vector3f(x + v.x, y + v.y, z + v.z); }
    Vector3f operator-(const Vector3f& v) const { return Vector3f(x - v.x, y - v.y, z - v.z); }
    Vector3f operator*(float s) const { return Vector3f(x * s, y * s, z * s); }
    Vector3f& operator+=(const Vector3f& v) { x += v.x; y += v.y; z += v.z; return *this; }

    float length() const { return sqrtf(x*x + y*y + z*z); }
    float length_xy() const { return sqrtf(x*x + y*y); }
};

// Environment parameters
struct EnvironmentParams {
    // Wind parameters
    float wind_speed_ms;        // Wind speed in m/s
    float wind_direction_deg;   // Wind direction (where wind comes FROM, 0=North)
    float wind_turbulence;      // Turbulence factor (0-1)
    float wind_gust_factor;     // Gust factor (1.0 = no gusts)

    // Wave parameters
    bool wave_enable;
    float wave_height_m;        // Significant wave height
    float wave_period_s;        // Wave period in seconds
    float wave_direction_deg;   // Wave direction
    float wave_spread_deg;      // Directional spread

    // Tide/current parameters
    float tide_speed_ms;        // Current speed
    float tide_direction_deg;   // Current direction

    // Convenience functions
    BeaufortScale get_beaufort() const;
    SeaState get_sea_state() const;
};

// Wingsail physical parameters
struct WingsailParams {
    WingsailType type;

    // Common parameters
    float sail_area_m2;         // Sail area in m^2
    float aspect_ratio;         // Aspect ratio (height/chord)
    float mass_kg;              // Wingsail mass

    // ROTATION mode parameters
    float rotation_angle_max_deg;   // Max rotation angle
    float rotation_angle_ideal_deg; // Ideal attack angle

    // FLAP mode parameters
    float flap_chord_ratio;     // Flap chord / total chord (typically 0.2-0.3)
    float flap_angle_max_deg;   // Max flap deflection
    float flap_angle_normal_deg;// Normal operating flap angle

    // FREE mode parameters
    float free_balance_point;   // Balance point ratio (0-1)
};

// Hull parameters
struct HullParams {
    float length_m;             // Hull length
    float beam_m;               // Hull width
    float draft_m;              // Draft depth
    float displacement_kg;      // Displacement
    float wetted_area_m2;       // Wetted surface area

    // Hydrodynamic coefficients
    float drag_coefficient;     // Cd
    float lift_coefficient;     // Cl (for hull lift)
    float added_mass_x;         // Added mass coefficient (surge)
    float added_mass_y;         // Added mass coefficient (sway)

    // Steering
    float rudder_area_m2;
    float rudder_max_angle_deg;
    float turning_circle_m;     // Minimum turning circle diameter
};

// Simulation state
struct SimState {
    // Position (NED frame, meters from origin)
    double lat_deg;
    double lon_deg;
    float altitude_m;

    // Velocity (NED frame, m/s)
    Vector3f velocity_ef;       // Earth frame velocity
    Vector3f velocity_bf;       // Body frame velocity
    Vector3f velocity_water;    // Velocity relative to water

    // Attitude (radians)
    float roll_rad;
    float pitch_rad;
    float yaw_rad;

    // Angular rates (rad/s)
    Vector3f gyro;

    // Accelerations
    Vector3f accel_bf;          // Body frame acceleration

    // Wingsail state
    float wingsail_angle_deg;   // Current wingsail angle (body frame)
    float flap_angle_deg;       // Current flap angle
    float normalized_control;   // Normalized control output (-1 to +1)

    // Apparent wind
    float apparent_wind_speed_ms;
    float apparent_wind_dir_deg;// Body frame, 0 = ahead

    // Performance metrics
    float vmg;                  // Velocity Made Good
    float heel_angle_deg;       // Heel angle
    float leeway_deg;           // Leeway angle
};

/*
    USV Sailboat Simulator Class

    Physical model for three wingsail types with realistic
    aerodynamic and hydrodynamic simulation.
*/
class SailboatUSV {
public:
    SailboatUSV();
    ~SailboatUSV() = default;

    // Initialization
    void init(WingsailType type = WingsailType::WINGSAIL_FLAP);
    void reset();

    // Configuration
    void set_wingsail_params(const WingsailParams& params);
    void set_hull_params(const HullParams& params);
    void set_environment(const EnvironmentParams& env);

    // Control inputs (normalized -1 to +1)
    void set_rudder_input(float rudder);        // Steering input
    void set_wingsail_input(float wingsail);    // Wingsail/flap input
    void set_throttle_input(float throttle);    // Motor throttle (if equipped)

    // Simulation step
    void update(float dt_s);

    // State access
    const SimState& get_state() const { return state_; }
    WingsailType get_wingsail_type() const { return wingsail_.type; }

    // Environment presets
    void set_beaufort(BeaufortScale beaufort);
    void set_sea_state(SeaState sea_state);

    // Convenience - set test scenario
    void set_scenario_upwind();
    void set_scenario_downwind();
    void set_scenario_beam_reach();
    void set_scenario_storm();

private:
    // Physical parameters
    WingsailParams wingsail_;
    HullParams hull_;
    EnvironmentParams env_;

    // Simulation state
    SimState state_;

    // Control inputs
    float rudder_input_;
    float wingsail_input_;
    float throttle_input_;

    // Internal state
    float wave_phase_;
    uint32_t sim_time_ms_;
    bool motor_enabled_;

    // Physics calculations
    void update_apparent_wind();
    void update_wingsail_forces(float& lift, float& drag, float& moment);
    void update_hull_forces(float& drag, float& side_force);
    void update_rudder_forces(float& force, float& moment);
    void update_wave_effects(float dt);
    void update_tide_effects();

    // Aerodynamic models for each wingsail type
    void calc_rotation_wingsail(float apparent_wind_speed, float apparent_wind_angle,
                                float wingsail_angle, float& lift, float& drag);
    void calc_flap_wingsail(float apparent_wind_speed, float apparent_wind_angle,
                            float flap_angle, float& lift, float& drag);
    void calc_free_wingsail(float apparent_wind_speed, float apparent_wind_angle,
                            float& lift, float& drag, float& equilibrium_angle);

    // Lift/drag lookup tables
    static constexpr int AERO_TABLE_SIZE = 19;
    static const float lift_curve_rotation_[AERO_TABLE_SIZE];
    static const float drag_curve_rotation_[AERO_TABLE_SIZE];
    static const float lift_curve_flap_[AERO_TABLE_SIZE];
    static const float drag_curve_flap_[AERO_TABLE_SIZE];

    // Helper functions
    float wrap_180(float angle) const;
    float wrap_360(float angle) const;
    float interpolate_curve(const float* curve, float angle_deg) const;
    float get_yaw_rate(float rudder, float speed) const;
};

// Utility functions
float beaufort_to_wind_speed(BeaufortScale beaufort);
float sea_state_to_wave_height(SeaState state);
BeaufortScale wind_speed_to_beaufort(float speed_ms);
SeaState wave_height_to_sea_state(float height_m);

} // namespace SITL_USV
