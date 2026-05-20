#include "Rover.h"

bool ModeRTL::_enter()
{
    // refuse RTL if home has not been set
    if (!AP::ahrs().home_is_set()) {
        return false;
    }

    // initialise waypoint navigation library
    g2.wp_nav.init(MAX(0.0f, g2.rtl_speed));

    // set target to the closest rally point or home
#if HAL_RALLY_ENABLED
    if (!g2.wp_nav.set_desired_location(g2.rally.calc_best_rally_or_home_location(rover.current_loc, ahrs.get_home().alt))) {
        return false;
    }
#else
    // set destination
    if (!g2.wp_nav.set_desired_location(ahrs.get_home())) {
        return false;
    }
#endif

    send_notification = true;
    _loitering = false;

    // OMNIX P5: snapshot heading + reset controller if frame is OMNIX
    _omni_yaw_state.initial_yaw = radians(ahrs.yaw_sensor * 0.01f);
    _omni_yaw_state.rc_yaw_integ = 0.0f;
    _omni_active = (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX);
    if (_omni_active) {
        g2.omni_ctrl.reset();
    }

    return true;
}

void ModeRTL::update()
{
    // OMNIX 4-thruster holonomic branch (P5)
    if (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX) {
        update_omnix_wp();
        return;
    }

    // determine if we should keep navigating
    if (!g2.wp_nav.reached_destination()) {
        // update navigation controller
        navigate_to_waypoint();
    } else {
        // send notification
        if (send_notification) {
            send_notification = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached destination");
        }

        // we have reached the destination
        // boats loiter, rovers stop
        if (!rover.is_boat()) {
            stop_vehicle();
        } else {
            // if not loitering yet, start loitering
            if (!_loitering) {
                _loitering = rover.mode_loiter.enter();
            }
            // update stop or loiter
            if (_loitering) {
                rover.mode_loiter.update();
            } else {
                stop_vehicle();
            }
        }

        // update distance to destination
        _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
    }
}

// get desired location
bool ModeRTL::get_desired_location(Location& destination) const
{
    if (g2.wp_nav.is_destination_valid()) {
        destination = g2.wp_nav.get_oa_destination();
        return true;
    }
    return false;
}

bool ModeRTL::reached_destination() const
{
    return g2.wp_nav.reached_destination();
}

// set desired speed in m/s
bool ModeRTL::set_desired_speed(float speed)
{
    return g2.wp_nav.set_speed_max(speed);
}

// OMNIX-only WP control for RTL. Called from update() when frame is OMNIX.
// Reads target from g2.wp_nav (home/rally set in _enter), yaw via shared
// omnix_compute_target_yaw helper, drives g2.omni_ctrl. Degrades to HOLD
// on lost position. On reached_destination: boats delegate to ModeLoiter
// (which has OMNIX branch from P4), rovers stop.
void ModeRTL::update_omnix_wp()
{
    g2.wp_nav.update(rover.G_Dt);

    if (g2.wp_nav.reached_destination()) {
        if (send_notification) {
            send_notification = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached destination");
        }
        if (rover.is_boat()) {
            if (!_loitering) {
                _loitering = rover.mode_loiter.enter();
            }
            if (_loitering) {
                rover.mode_loiter.update();
            } else {
                g2.motors.set_throttle(0.0f);
                g2.motors.set_lateral(0.0f);
                g2.motors.set_steering(0.0f);
            }
        } else {
            g2.motors.set_throttle(0.0f);
            g2.motors.set_lateral(0.0f);
            g2.motors.set_steering(0.0f);
        }
        _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
        return;
    }

    // --- Position estimate required ---
    Vector3f pos_ned;
    if (!ahrs.get_relative_position_NED_origin_float(pos_ned)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        rover.set_mode(rover.mode_hold, ModeReason::EKF_FAILSAFE);
        return;
    }

    if (!g2.wp_nav.is_destination_valid()) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    Location origin;
    if (!ahrs.get_origin(origin)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }
    const Vector2f target_pos_ned = origin.get_distance_NE(g2.wp_nav.get_destination());

    const float target_yaw = omnix_compute_target_yaw(_omni_yaw_state, rover.G_Dt);

    g2.omni_ctrl.set_target(target_pos_ned, target_yaw);
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
}
