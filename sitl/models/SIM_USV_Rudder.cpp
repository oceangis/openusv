/*
  USV Single Thruster + Rudder simulator class
*/

#include "SIM_USV_Rudder.h"
#include <AP_Math/AP_Math.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

USV_Rudder::USV_Rudder(const char *frame_str) :
    USV_Base(frame_str)
{
    mass_ = 5.0f;
    max_thrust_ = 50.0f;
    hull_drag_coeff_ = 0.5f;
    steering_angle_max_ = 35.0f;
    turning_circle_ = 1.8f;
    printf("USV Single Thruster + Rudder Simulation Started\n");
}

void USV_Rudder::update_propulsion(const struct sitl_input &input, float delta_time)
{
    // read steering and throttle servo outputs
    const float steering = input.servos[RUDDER_CH] ? normalise_servo_input(input.servos[RUDDER_CH]) : 0;
    const float throttle = input.servos[THROTTLE_CH] ? normalise_servo_input(input.servos[THROTTLE_CH]) : 0;

    // speed in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef_water_;
    float speed = velocity_body.x;

    // yaw rate from rudder steering
    float yaw_rate = get_yaw_rate_from_steering(steering, speed);

    // set gyro (base class adds wave_gyro_)
    gyro = Vector3f(0, 0, radians(yaw_rate));

    // thrust force
    float thrust = throttle * max_thrust_;

    // hull drag
    float hull_drag = sq(speed) * hull_drag_coeff_;
    if (!is_positive(speed)) {
        hull_drag *= -1.0f;
    }

    // body frame acceleration
    accel_body = Vector3f((thrust - hull_drag) / mass_, 0, 0);

    // add centripetal acceleration
    accel_body.y += radians(yaw_rate) * speed;
}

} // namespace SITL
