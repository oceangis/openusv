#include "Rover.h"

bool ModeLoiter::_enter()
{
    // set _destination to reasonable stopping point
    if (!g2.wp_nav.get_stopping_location(_destination)) {
        return false;
    }

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;

    // OMNIX P4: snapshot heading + reset controller if this Loiter entry is on OMNIX
    _omni_yaw_state.initial_yaw = radians(ahrs.yaw_sensor * 0.01f);
    _omni_yaw_state.rc_yaw_integ = 0.0f;
    _omni_active = (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX);
    if (_omni_active) {
        g2.omni_ctrl.reset();
    }

    return true;
}

void ModeLoiter::update()
{
    // OMNIX 4-thruster holonomic branch (P4)
    if (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX) {
        update_omnix();
        return;
    }

    // get distance (in meters) to destination
    _distance_to_destination = rover.current_loc.get_distance(_destination);

    const float loiter_radius = g2.sailboat.tack_enabled() ? g2.sailboat.get_loiter_radius() : g2.loit_radius;

    // if within loiter radius slew desired speed towards zero and use existing desired heading
    if (_distance_to_destination <= loiter_radius) {
        // sailboats should not stop unless motoring
        const float desired_speed_within_radius = g2.sailboat.tack_enabled() ? 0.1f : 0.0f;
        _desired_speed = attitude_control.get_desired_speed_accel_limited(desired_speed_within_radius, rover.G_Dt);

        // if we have a sail but not trying to use it then point into the wind
        if (!g2.sailboat.tack_enabled() && g2.sailboat.sail_enabled()) {
            _desired_yaw_cd = degrees(g2.windvane.get_true_wind_direction_rad()) * 100.0f;
        }
    } else {
        // P controller with hard-coded gain to convert distance to desired speed
        _desired_speed = MIN((_distance_to_destination - loiter_radius) * g2.loiter_speed_gain, g2.wp_nav.get_default_speed());

        // calculate bearing to destination
        _desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
        float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
        // if destination is behind vehicle, reverse towards it
        if ((fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) || g2.loit_type == 2) {
            _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
            yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
            _desired_speed = -_desired_speed;
        }

        // reduce desired speed if yaw_error is large
        // 45deg of error reduces speed to 75%, 90deg of error reduces speed to 50%
        float yaw_error_ratio = 1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
        _desired_speed *= yaw_error_ratio;
    }

    // 0 turn rate is no limit
    float turn_rate = 0.0;

    // make sure sailboats don't try and sail directly into the wind
    if (g2.sailboat.use_indirect_route(_desired_yaw_cd)) {
        _desired_yaw_cd = g2.sailboat.calc_heading(_desired_yaw_cd);
        if (g2.sailboat.tacking()) {
            // use pivot turn rate for tacks
            turn_rate = g2.wp_nav.get_pivot_rate();
        }
    }

    // run steering and throttle controllers
    calc_steering_to_heading(_desired_yaw_cd, turn_rate);
    calc_throttle(_desired_speed, true);
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}

// OMNIX-only loiter: station-keep at _destination using g2.omni_ctrl.
// Called from ModeLoiter::update() when frame is OMNIX. Heading is always
// LOCK_INITIAL (snapshot at _enter) — Loiter is stationary, no path tangent.
// Use POSHOLD for other yaw strategies.
void ModeLoiter::update_omnix()
{
    // --- Position estimate required ---
    Vector3f pos_ned;
    if (!ahrs.get_relative_position_NED_origin_float(pos_ned)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        rover.set_mode(rover.mode_hold, ModeReason::EKF_FAILSAFE);
        return;
    }

    // --- Location -> NED ---
    Location origin;
    if (!ahrs.get_origin(origin)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }
    const Vector2f target_pos_ned = origin.get_distance_NE(_destination);

    // --- Drive controller (LOCK_INITIAL yaw — stationary) ---
    g2.omni_ctrl.set_target(target_pos_ned, _omni_yaw_state.initial_yaw);
    g2.omni_ctrl.update(rover.G_Dt);

    float fwd, lat, steer_norm;
    if (!g2.omni_ctrl.get_outputs(fwd, lat, steer_norm)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        rover.set_mode(rover.mode_hold, ModeReason::EKF_FAILSAFE);
        return;
    }

    g2.motors.set_throttle(fwd * 100.0f);
    g2.motors.set_lateral(lat * 100.0f);
    g2.motors.set_steering(steer_norm * 4500.0f, false);

    // Update reporting field for GCS
    _distance_to_destination = rover.current_loc.get_distance(_destination);
}
