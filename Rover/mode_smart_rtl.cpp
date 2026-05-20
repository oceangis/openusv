#include "Rover.h"


#if MODE_SMARTRTL_ENABLED

bool ModeSmartRTL::_enter()
{
    // SmartRTL requires EKF (not DCM)
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    // refuse to enter SmartRTL if smart RTL's home has not been set
    if (!g2.smart_rtl.is_active()) {
        return false;
    }

    // initialise waypoint navigation library
    g2.wp_nav.init(MAX(0, g2.rtl_speed));

    // set desired location to reasonable stopping point
    if (!g2.wp_nav.set_desired_location_to_stopping_location()) {
        return false;
    }

    // init state
    smart_rtl_state = SmartRTLState::WaitForPathCleanup;
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

void ModeSmartRTL::update()
{
    switch (smart_rtl_state) {
        case SmartRTLState::WaitForPathCleanup:
            // check if return path is computed and if yes, begin journey home
            if (g2.smart_rtl.request_thorough_cleanup()) {
                smart_rtl_state = SmartRTLState::PathFollow;
                _load_point = true;
            }
            // Note: this may lead to an unnecessary 20ms slow down of the vehicle (but it is unlikely)
            stop_vehicle();
            break;

        case SmartRTLState::PathFollow:
            // load point if required
            if (_load_point) {
                Vector3p dest_NED;
                if (!g2.smart_rtl.pop_point(dest_NED)) {
                    // if not more points, we have reached home
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached destination");
                    smart_rtl_state = SmartRTLState::StopAtHome;
                    break;
                } else {
                    // peek at the next point.  this can fail if the IO task currently has the path semaphore
                    Vector3p next_dest_NED;
                    if (g2.smart_rtl.peek_point(next_dest_NED)) {
                        if (!g2.wp_nav.set_desired_location_NED(dest_NED.tofloat(), next_dest_NED.tofloat())) {
                            // this should never happen because the EKF origin should already be set
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SmartRTL: failed to set destination");
                            smart_rtl_state = SmartRTLState::Failure;
                            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                        }
                    } else {
                        // no next point so add only immediate point
                        if (!g2.wp_nav.set_desired_location_NED(dest_NED.tofloat())) {
                            // this should never happen because the EKF origin should already be set
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SmartRTL: failed to set destination");
                            smart_rtl_state = SmartRTLState::Failure;
                            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                        }
                    }
                }
                _load_point = false;
            }
            // OMNIX 4-thruster holonomic branch (P5)
            if (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX) {
                update_omnix_wp();
            } else {
                // update navigation controller (differential-drive path)
                navigate_to_waypoint();
            }

            // check if we've reached the next point
            if (g2.wp_nav.reached_destination()) {
                _load_point = true;
            }
            break;

        case SmartRTLState::StopAtHome:
        case SmartRTLState::Failure:
            _reached_destination = true;
            // we have reached the destination
            // boats loiters, rovers stop
            if (!rover.is_boat()) {
               stop_vehicle();
            } else {
                // if not loitering yet, start loitering
                if (!_loitering) {
                    _loitering = rover.mode_loiter.enter();
                }
                if (_loitering) {
                    rover.mode_loiter.update();
                } else {
                    stop_vehicle();
               }
            }
            break;
    }
}

// get desired location
bool ModeSmartRTL::get_desired_location(Location& destination) const
{
    switch (smart_rtl_state) {
    case SmartRTLState::WaitForPathCleanup:
        return false;
    case SmartRTLState::PathFollow:
        if (g2.wp_nav.is_destination_valid()) {
            destination = g2.wp_nav.get_destination();
            return true;
        }
        return false;
    case SmartRTLState::StopAtHome:
    case SmartRTLState::Failure:
        return false;
    }
    // should never reach here but just in case
    return false;
}

// set desired speed in m/s
bool ModeSmartRTL::set_desired_speed(float speed)
{
    return g2.wp_nav.set_speed_max(speed);
}

// save current position for use by the smart_rtl flight mode
void ModeSmartRTL::save_position()
{
    const bool save_pos = (rover.control_mode != &rover.mode_smartrtl);
    g2.smart_rtl.update(true, save_pos);
}


// OMNIX-only WP control for SmartRTL PathFollow state. Called when
// frame is OMNIX. Replaces navigate_to_waypoint() with g2.omni_ctrl
// drive toward the current popped path point.
void ModeSmartRTL::update_omnix_wp()
{
    g2.wp_nav.update(rover.G_Dt);

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

#endif  // MODE_SMARTRTL_ENABLED
