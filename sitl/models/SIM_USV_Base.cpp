/*
  USV Base simulator class - common water physics for all USV boat types
*/

#include "SIM_USV_Base.h"
#include <AP_Math/AP_Math.h>
#include <string.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

// very roughly sort of a stability factors for waves
#define WAVE_ANGLE_GAIN 1
#define WAVE_HEAVE_GAIN 1

USV_Base::USV_Base(const char *frame_str) :
    Aircraft(frame_str),
    mass_(2.0f),
    steering_angle_max_(35.0f),
    turning_circle_(1.8f),
    max_thrust_(50.0f),
    hull_drag_coeff_(0.5f),
    wave_heave_(0.0f),
    wave_phase_(0.0f)
{
    lock_step_scheduled = true;
}

// return turning circle (diameter) in meters for steering angle proportion in the range -1 to +1
float USV_Base::get_turn_circle(float steering) const
{
    if (is_zero(steering)) {
        return 0;
    }
    return turning_circle_ * sinf(radians(steering_angle_max_)) / sinf(radians(steering * steering_angle_max_));
}

// return yaw rate in deg/sec given a steering input (in the range -1 to +1) and speed in m/s
float USV_Base::get_yaw_rate_from_steering(float steering, float speed) const
{
    if (is_zero(steering) || is_zero(speed)) {
        return 0.0f;
    }
    float d = get_turn_circle(steering);
    float c = M_PI * d;
    float t = c / speed;
    float rate = 360.0f / t;
    return rate;
}

// simulate basic waves / swell (from SIM_Sailboat.cpp)
void USV_Base::update_wave(float delta_time)
{
    const float wave_heading = sitl->wave.direction;
    const float wave_speed = sitl->wave.speed;
    const float wave_lenght = sitl->wave.length;
    const float wave_amp = sitl->wave.amp;

    // apply rate proportional to error between boat angle and water angle
    // this gives a 'stability' effect
    float r, p, y;
    dcm.to_euler(&r, &p, &y);

    // if not armed don't do waves, to allow gyro init
    if (sitl->wave.enable == 0 || !hal.util->get_soft_armed() || is_zero(wave_amp)) {
        wave_gyro_ = Vector3f(-r, -p, 0.0f) * WAVE_ANGLE_GAIN;
        wave_heave_ = -velocity_ef.z * WAVE_HEAVE_GAIN;
        wave_phase_ = 0.0f;
        return;
    }

    // calculate the boat speed in the direction of the wave
    const float boat_speed = velocity_ef.x * sinf(radians(wave_heading)) + velocity_ef.y * cosf(radians(wave_heading));

    // update the wave phase
    const float aprarent_wave_distance = (wave_speed - boat_speed) * delta_time;
    const float apparent_wave_phase_change = (aprarent_wave_distance / wave_lenght) * M_2PI;

    wave_phase_ += apparent_wave_phase_change;
    wave_phase_ = wrap_2PI(wave_phase_);

    // calculate the angles at this phase on the wave
    // use basic sine wave, dy/dx of sine = cosine
    // atan( cosine ) = wave angle
    const float wave_slope = (wave_amp * 0.5f) * (M_2PI / wave_lenght) * cosf(wave_phase_);
    const float wave_angle = atanf(wave_slope);

    // convert wave angle to vehicle frame
    const float heading_dif = wave_heading - y;
    float angle_error_x = (sinf(heading_dif) * wave_angle) - r;
    float angle_error_y = (cosf(heading_dif) * wave_angle) - p;

    // apply gain
    wave_gyro_.x = angle_error_x * WAVE_ANGLE_GAIN;
    wave_gyro_.y = angle_error_y * WAVE_ANGLE_GAIN;
    wave_gyro_.z = 0.0f;

    // calculate wave height (NED)
    if (sitl->wave.enable == 2) {
        wave_heave_ = (wave_slope - velocity_ef.z) * WAVE_HEAVE_GAIN;
    } else {
        wave_heave_ = 0.0f;
    }
}

// integrate velocity and position from accel_body
void USV_Base::integrate_state(float delta_time)
{
    // now in earth frame
    // remove roll and pitch effects from waves
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    Matrix3f temp_dcm;
    temp_dcm.from_euler(0.0f, 0.0f, y);
    Vector3f accel_earth = temp_dcm * accel_body;

    // we are on the water, so our vertical accel is zero (plus wave heave)
    accel_earth.z = 0 + wave_heave_;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // tide calcs
    Vector3f tide_velocity_ef;
    if (hal.util->get_soft_armed() && !is_zero(sitl->tide.speed)) {
        tide_velocity_ef.x = -cosf(radians(sitl->tide.direction)) * sitl->tide.speed;
        tide_velocity_ef.y = -sinf(radians(sitl->tide.direction)) * sitl->tide.speed;
        tide_velocity_ef.z = 0.0f;
    }

    // new velocity vector
    velocity_ef_water_ += accel_earth * delta_time;
    velocity_ef = velocity_ef_water_ + tide_velocity_ef;

    // new position vector
    position += (velocity_ef * delta_time).todouble();

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

/*
  update the USV simulation by one time step
 */
void USV_Base::update(const struct sitl_input &input)
{
    // update wind
    update_wind(input);

    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    // let derived class compute propulsion forces
    update_propulsion(input, delta_time);

    // add wave effects to gyro
    gyro += wave_gyro_;

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // integrate velocity, position, and recompute accel_body for accelerometer
    integrate_state(delta_time);

    // update wave calculations
    update_wave(delta_time);
}

} // namespace SITL
