/*
  USV Wingsail simulator class
  Handles both rotation-only wingsail and wingsail+flap variants
*/

#include "SIM_USV_Wingsail.h"
#include <AP_Math/AP_Math.h>
#include <string.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

// aerodynamic lookup tables (18 points at 10 deg intervals, 0-170 deg)
const float USV_Wingsail::lift_curve_[18] = {
    0.0f, 0.5f, 0.9f, 1.2f, 1.3f, 1.3f, 1.2f, 1.0f,
    0.8f, 0.6f, 0.4f, 0.2f, 0.1f, 0.0f, -0.1f, -0.1f, -0.05f, 0.0f
};
const float USV_Wingsail::drag_curve_[18] = {
    0.01f, 0.02f, 0.04f, 0.06f, 0.10f, 0.15f, 0.22f, 0.30f,
    0.40f, 0.50f, 0.60f, 0.70f, 0.78f, 0.85f, 0.90f, 0.93f, 0.95f, 0.96f
};

USV_Wingsail::USV_Wingsail(const char *frame_str) :
    USV_Base(frame_str),
    has_flap_(strstr(frame_str, "flap") != nullptr),
    sail_area_(1.0f)
{
    mass_ = 3.0f;
    hull_drag_coeff_ = 0.4f;
    steering_angle_max_ = 35.0f;
    turning_circle_ = 1.8f;

    if (has_flap_) {
        printf("USV Wingsail + Flap Simulation Started\n");
    } else {
        printf("USV Wingsail Simulation Started\n");
    }
}

// calculate the lift and drag given an apparent wind speed in m/s and angle-of-attack in degrees
void USV_Wingsail::calc_lift_and_drag(float wind_speed, float angle_of_attack_deg, float &lift, float &drag) const
{
    const uint16_t index_width_deg = 10;
    const uint8_t index_max = ARRAY_SIZE(lift_curve_) - 1;

    // convert to expected range
    angle_of_attack_deg = wrap_180(angle_of_attack_deg);

    // assume a symmetrical airfoil
    const float aoa = fabs(angle_of_attack_deg);

    // check extremes
    if (aoa <= 0.0f) {
        lift = lift_curve_[0];
        drag = drag_curve_[0];
    } else if (aoa >= index_max * index_width_deg) {
        lift = lift_curve_[index_max];
        drag = drag_curve_[index_max];
    } else {
        uint8_t index = constrain_int16(aoa / index_width_deg, 0, index_max);
        float remainder = aoa - (index * index_width_deg);
        lift = linear_interpolate(lift_curve_[index], lift_curve_[index+1], remainder, 0.0f, (float)index_width_deg);
        drag = linear_interpolate(drag_curve_[index], drag_curve_[index+1], remainder, 0.0f, (float)index_width_deg);
    }

    // apply scaling by wind speed
    lift *= wind_speed * sail_area_;
    drag *= wind_speed * sail_area_;

    if (is_negative(angle_of_attack_deg)) {
        // invert lift for negative aoa
        lift *= -1;
    }
}

void USV_Wingsail::update_propulsion(const struct sitl_input &input, float delta_time)
{
    // read steering from rudder servo
    const float steering = input.servos[RUDDER_CH] ? normalise_servo_input(input.servos[RUDDER_CH]) : 0;

    // read wingsail angle: servo maps (1500 +/- 500) to +/- 90 degrees
    const float wingsail_angle = constrain_float(
        (input.servos[WINGSAIL_CH] - 1500) / 500.0f * 90.0f, -90.0f, 90.0f);

    // calculate apparent wind in earth-frame
    // Note: SITL wind_ef is the direction wind is travelling TO
    // apparent wind = boat velocity - wind velocity
    Vector3f wind_apparent_ef = velocity_ef - wind_ef;
    const float wind_apparent_dir_ef = degrees(atan2f(wind_apparent_ef.y, wind_apparent_ef.x));
    const float wind_apparent_speed = safe_sqrt(sq(wind_apparent_ef.x) + sq(wind_apparent_ef.y));

    // get yaw from dcm
    float roll, pitch, yaw;
    dcm.to_euler(&roll, &pitch, &yaw);

    // apparent wind direction in body frame
    const float wind_apparent_dir_bf = wrap_180(wind_apparent_dir_ef - degrees(yaw));

    // calculate angle-of-attack
    float aoa_deg = 0.0f;
    if (has_flap_) {
        // wingsail with trailing-edge flap
        const float flap_angle = constrain_float(
            (input.servos[FLAP_CH] - 1500) / 500.0f * 30.0f, -30.0f, 30.0f);
        aoa_deg = wind_apparent_dir_bf - wingsail_angle - flap_angle * 0.5f;
    } else {
        // directly actuated wing (rotation only)
        aoa_deg = wind_apparent_dir_bf - wingsail_angle;
    }

    // calculate lift and drag forces
    float lift_wf, drag_wf;
    calc_lift_and_drag(wind_apparent_speed, aoa_deg, lift_wf, drag_wf);

    // rotate lift and drag from wind frame into body frame
    const float sin_rot_rad = sinf(radians(wind_apparent_dir_bf));
    const float cos_rot_rad = cosf(radians(wind_apparent_dir_bf));
    const float force_fwd = (lift_wf * sin_rot_rad) - (drag_wf * cos_rot_rad);

    // speed in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef_water_;
    float speed = velocity_body.x;

    // yaw rate from rudder steering
    float yaw_rate = get_yaw_rate_from_steering(steering, speed);

    // set gyro
    gyro = Vector3f(0, 0, radians(yaw_rate));

    // hull drag
    float hull_drag = sq(speed) * hull_drag_coeff_;
    if (!is_positive(speed)) {
        hull_drag *= -1.0f;
    }

    // body frame acceleration
    accel_body = Vector3f((force_fwd - hull_drag) / mass_, 0, 0);

    // add centripetal acceleration
    accel_body.y += radians(yaw_rate) * speed;

    // set RPM and airspeed from apparent wind speed (for wind vane backend)
    rpm[0] = wind_apparent_speed;
    airspeed_pitot = wind_apparent_speed;
}

} // namespace SITL
