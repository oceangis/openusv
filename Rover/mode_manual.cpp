#include "Rover.h"

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}

void ModeManual::update()
{
    float desired_steering, desired_throttle, desired_lateral;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    get_pilot_desired_lateral(desired_lateral);

    // apply manual steering expo
    desired_steering = 4500.0 * input_expo(desired_steering / 4500, g2.manual_steering_expo);

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);
    }

    // walking robots support roll, pitch and walking_height
    float desired_roll, desired_pitch, desired_walking_height;
    get_pilot_desired_roll_and_pitch(desired_roll, desired_pitch);
    get_pilot_desired_walking_height(desired_walking_height);
    g2.motors.set_roll(desired_roll);
    g2.motors.set_pitch(desired_pitch);
    g2.motors.set_walking_height(desired_walking_height);

    // set sailboat sails
    g2.sailboat.set_pilot_desired_mainsail();

    // X-type vectored boats: optional heading assist (ArduSub-style)
    // Only enabled for FRAME_TYPE_BOAT_VECTORED_X
    if (g2.frame_type == (uint8_t)AP_MotorsUGV::FRAME_TYPE_BOAT_VECTORED_X) {
        // use heading assist instead of direct steering
        update_heading_assist();
        // set throttle and lateral directly
        g2.motors.set_throttle(desired_throttle);
        g2.motors.set_lateral(desired_lateral);
    } else {
        // normal manual mode for other vehicle types
        g2.motors.set_throttle(desired_throttle);
        g2.motors.set_steering(desired_steering, (g2.manual_options & ManualOptions::SPEED_SCALING));
        g2.motors.set_lateral(desired_lateral);
    }
}

// Heading assist for X-type vectored boats
// Implements ArduSub-style heading hold: 250ms smooth decel + absolute heading lock
void ModeManual::update_heading_assist()
{
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
