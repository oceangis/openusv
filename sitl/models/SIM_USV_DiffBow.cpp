/*
  USV Differential Thrust + Bow Thruster simulator class
*/

#include "SIM_USV_DiffBow.h"
#include <AP_Math/AP_Math.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

USV_DiffBow::USV_DiffBow(const char *frame_str) :
    USV_DiffThrust(frame_str),
    bow_thrust_max_(30.0f),
    bow_arm_(0.5f)
{
    printf("USV Differential + Bow Thruster Simulation Started\n");
}

void USV_DiffBow::update_propulsion(const struct sitl_input &input, float delta_time)
{
    // get base differential thrust forces
    USV_DiffThrust::update_propulsion(input, delta_time);

    // read bow thruster servo
    const float bow_input = input.servos[BOW_THRUSTER_CH] ? normalise_servo_input(input.servos[BOW_THRUSTER_CH]) : 0;
    if (is_zero(bow_input)) {
        return;
    }

    // lateral force from bow thruster
    float lateral_force = bow_input * bow_thrust_max_;
    accel_body.y += lateral_force / mass_;

    // yaw moment from bow thruster at bow_arm_ forward of CG
    // simplified moment of inertia: I_zz ≈ mass * (turning_circle/4)^2
    float r_gyration = turning_circle_ * 0.25f;
    float I_zz = mass_ * sq(r_gyration);
    // torque / I_zz = angular accel (rad/s^2), integrate to get rate delta
    float yaw_accel = (lateral_force * bow_arm_) / I_zz;
    gyro.z += yaw_accel * delta_time;
}

} // namespace SITL
