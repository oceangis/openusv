/*
   PosHold Mode - Position Hold mode for vectored thrust boats

   This mode is specifically designed for X-type vectored boats (FRAME_TYPE_BOAT_VECTORED_X).
   It provides ArduSub-style control with:
   - Independent position hold (GPS-based)
   - Heading hold with 250ms smooth deceleration
   - Body-frame velocity input (forward/lateral sticks)

   Based on ArduSub's PosHold mode implementation.
*/

#include "Rover.h"

bool ModePosHold::_enter()
{
    // Only allow entry for X-type vectored boats
    if (g2.frame_type != (uint8_t)AP_MotorsUGV::FRAME_TYPE_BOAT_VECTORED_X) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PosHold: Only for X-type boats");
        return false;
    }

    // check GPS position is available
    if (!rover.ekf_position_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PosHold: Need GPS lock");
        return false;
    }

    // set target position to current position
    _position_target = rover.current_loc;

    // initialize desired speeds to zero
    _desired_forward_speed = 0.0f;
    _desired_lateral_speed = 0.0f;

    // initialize heading to current heading
    _heading_target_cd = ahrs.yaw_sensor;
    _last_pilot_yaw_input_ms = AP_HAL::millis();

    gcs().send_text(MAV_SEVERITY_INFO, "PosHold: X-type boat mode active");

    return true;
}

void ModePosHold::update()
{
    // position/velocity control
    control_position();

    // heading control (ArduSub-style)
    control_heading();
}

void ModePosHold::control_position()
{
    // get pilot input (forward/lateral channels)
    float pilot_forward = 0.0f;
    float pilot_lateral = 0.0f;

    // get forward input from throttle channel
    if (channel_throttle != nullptr) {
        pilot_forward = channel_throttle->norm_input_dz();
    }

    // get lateral input
    get_pilot_desired_lateral(pilot_lateral);

    // convert to speed targets (scale by max speed)
    const float max_speed = g2.wp_nav.get_default_speed();
    _desired_forward_speed = pilot_forward * max_speed;
    _desired_lateral_speed = pilot_lateral * max_speed;

    // if pilot has input, update position target
    if (!is_zero(_desired_forward_speed) || !is_zero(_desired_lateral_speed)) {
        // pilot is commanding movement - update position target
        // integrate velocity into position target

        // convert body-frame velocity to earth-frame velocity
        const float yaw_rad = ahrs.get_yaw_rad();
        const float cos_yaw = cosf(yaw_rad);
        const float sin_yaw = sinf(yaw_rad);

        // body to earth frame: NE = R(yaw) * [forward; lateral]
        const float vel_n = _desired_forward_speed * cos_yaw - _desired_lateral_speed * sin_yaw;
        const float vel_e = _desired_forward_speed * sin_yaw + _desired_lateral_speed * cos_yaw;

        // update position target (integrate velocity)
        _position_target.offset(vel_n * rover.G_Dt, vel_e * rover.G_Dt);

        // set outputs directly from pilot input (feedforward)
        float forward_out, lateral_out;
        attitude_control.get_vectored_out_speed(_desired_forward_speed,
                                                 _desired_lateral_speed,
                                                 g2.motors.limit.throttle_lower || g2.motors.limit.throttle_upper,
                                                 g.speed_cruise,
                                                 g.throttle_cruise * 0.01f,
                                                 rover.G_Dt,
                                                 forward_out,
                                                 lateral_out);

        g2.motors.set_throttle(forward_out);
        g2.motors.set_lateral(lateral_out);

    } else {
        // no pilot input - hold current position using GPS feedback

        // calculate position error (earth frame)
        const float bearing_to_target = rover.current_loc.get_bearing_to(_position_target);
        const float distance_to_target = rover.current_loc.get_distance(_position_target);

        // position error in NE frame
        float pos_error_n = distance_to_target * cosf(bearing_to_target);
        float pos_error_e = distance_to_target * sinf(bearing_to_target);

        // simple P controller: velocity = position_error * gain
        // Using same gain as loiter mode for consistency
        const float pos_p_gain = constrain_float(g2.loiter_speed_gain, 0.1f, 2.0f);
        float vel_target_n = pos_error_n * pos_p_gain;
        float vel_target_e = pos_error_e * pos_p_gain;

        // limit velocity target to 50% of max speed when holding position
        const float vel_max_hold = max_speed * 0.5f;
        const float vel_mag = sqrtf(sq(vel_target_n) + sq(vel_target_e));
        if (vel_mag > vel_max_hold) {
            vel_target_n *= vel_max_hold / vel_mag;
            vel_target_e *= vel_max_hold / vel_mag;
        }

        // convert earth-frame velocity target to body-frame
        const float yaw_rad = ahrs.get_yaw_rad();
        const float cos_yaw = cosf(yaw_rad);
        const float sin_yaw = sinf(yaw_rad);

        // earth to body frame: [forward; lateral] = R(-yaw) * [N; E]
        float forward_speed_target = vel_target_n * cos_yaw + vel_target_e * sin_yaw;
        float lateral_speed_target = -vel_target_n * sin_yaw + vel_target_e * cos_yaw;

        // call velocity controller
        float forward_out, lateral_out;
        attitude_control.get_vectored_out_speed(forward_speed_target,
                                                 lateral_speed_target,
                                                 g2.motors.limit.throttle_lower || g2.motors.limit.throttle_upper,
                                                 g.speed_cruise,
                                                 g.throttle_cruise * 0.01f,
                                                 rover.G_Dt,
                                                 forward_out,
                                                 lateral_out);

        g2.motors.set_throttle(forward_out);
        g2.motors.set_lateral(lateral_out);
    }
}

void ModePosHold::control_heading()
{
    // ArduSub-style heading hold:
    // - With yaw input: use rate control
    // - First 250ms after releasing yaw: smoothly decelerate to zero
    // - After 250ms: lock to absolute heading

    const uint32_t tnow = AP_HAL::millis();

    // get yaw input from pilot (using roll channel for X-frame boats)
    float yaw_input = 0.0f;
    if (channel_roll != nullptr) {
        yaw_input = channel_roll->norm_input_dz();
    }

    if (!is_zero(yaw_input)) {
        // pilot is commanding yaw - use rate control
        const float target_yaw_rate = yaw_input * radians(g2.acro_turn_rate);

        float steering_out = attitude_control.get_steering_out_rate(
            target_yaw_rate,
            g2.motors.limit.steer_left,
            g2.motors.limit.steer_right,
            rover.G_Dt
        );

        // convert to centi-degrees for motors
        g2.motors.set_steering(steering_out * 4500.0f);

        // update heading target to current heading
        _heading_target_cd = ahrs.yaw_sensor;
        _last_pilot_yaw_input_ms = tnow;

    } else {
        // no yaw input from pilot

        if (tnow < _last_pilot_yaw_input_ms + 250) {
            // first 250ms after releasing yaw stick: smoothly decelerate to zero
            float steering_out = attitude_control.get_steering_out_rate(
                0.0f,  // target rate = 0
                g2.motors.limit.steer_left,
                g2.motors.limit.steer_right,
                rover.G_Dt
            );

            g2.motors.set_steering(steering_out * 4500.0f);

            // update heading target during deceleration
            _heading_target_cd = ahrs.yaw_sensor;

        } else {
            // after 250ms: lock to absolute heading
            calc_steering_to_heading(_heading_target_cd);
        }
    }
}

bool ModePosHold::get_desired_location(Location& destination) const
{
    destination = _position_target;
    return true;
}

float ModePosHold::get_distance_to_destination() const
{
    return rover.current_loc.get_distance(_position_target);
}
