/*
  USV Differential Thrust simulator class
*/

#include "SIM_USV_DiffThrust.h"
#include <AP_Math/AP_Math.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

USV_DiffThrust::USV_DiffThrust(const char *frame_str) :
    USV_Base(frame_str),
    motor_arm_(0.3f),
    skid_turn_rate_(140.0f)
{
    mass_ = 5.0f;
    max_thrust_ = 50.0f;
    hull_drag_coeff_ = 0.5f;
    printf("USV Differential Thrust Simulation Started\n");
}

void USV_DiffThrust::update_propulsion(const struct sitl_input &input, float delta_time)
{
    // read left and right motor servo outputs
    const float motor_left = input.servos[USV_MOTOR_LEFT_CH] ? normalise_servo_input(input.servos[USV_MOTOR_LEFT_CH]) : 0;
    const float motor_right = input.servos[USV_MOTOR_RIGHT_CH] ? normalise_servo_input(input.servos[USV_MOTOR_RIGHT_CH]) : 0;

    // combined throttle and steering
    float throttle = 0.5f * (motor_left + motor_right);
    float steering = motor_left - motor_right;

    // speed in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef_water_;
    float speed = velocity_body.x;

    // yaw rate from differential thrust
    float yaw_rate = constrain_float(steering * skid_turn_rate_, -360.0f, 360.0f);

    // set gyro (derived class sets base, base class adds wave_gyro_)
    gyro = Vector3f(0, 0, radians(yaw_rate));

    // thrust force
    float thrust_force = throttle * max_thrust_;

    // hull drag
    float hull_drag = sq(speed) * hull_drag_coeff_;
    if (!is_positive(speed)) {
        hull_drag *= -1.0f;
    }

    // body frame acceleration
    accel_body = Vector3f((thrust_force - hull_drag) / mass_, 0, 0);

    // add centripetal acceleration
    accel_body.y += radians(yaw_rate) * speed;
}

} // namespace SITL
