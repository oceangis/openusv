/*
  USV X-Frame quad thruster simulator class

  Motor layout (top view):
      M1(FL)     M2(FR)
         \       /
          \     /
           [CG]
          /     \
         /       \
      M3(BL)     M4(BR)
*/

#include "SIM_USV_XFrame.h"
#include <AP_Math/AP_Math.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

USV_XFrame::USV_XFrame(const char *frame_str) :
    USV_Base(frame_str),
    arm_(0.3f),
    mount_angle_(45.0f),
    thrust_per_motor_(25.0f)
{
    mass_ = 8.0f;
    hull_drag_coeff_ = 0.5f;
    printf("USV X-Frame Quad Thruster Simulation Started\n");
}

void USV_XFrame::update_propulsion(const struct sitl_input &input, float delta_time)
{
    // read 4 motor servo outputs (-1 to +1)
    const float m1 = input.servos[MOTOR_FL_CH] ? normalise_servo_input(input.servos[MOTOR_FL_CH]) : 0;
    const float m2 = input.servos[MOTOR_FR_CH] ? normalise_servo_input(input.servos[MOTOR_FR_CH]) : 0;
    const float m3 = input.servos[MOTOR_BL_CH] ? normalise_servo_input(input.servos[MOTOR_BL_CH]) : 0;
    const float m4 = input.servos[MOTOR_BR_CH] ? normalise_servo_input(input.servos[MOTOR_BR_CH]) : 0;

    // precompute trig for mount angle
    const float cos_mount = cosf(radians(mount_angle_));
    const float sin_mount = sinf(radians(mount_angle_));

    // force mixing (X-frame configuration)
    // forward: all motors contribute via cos(mount_angle)
    const float force_x = (m1 + m2 + m3 + m4) * cos_mount * thrust_per_motor_;

    // lateral: differential left/right via sin(mount_angle)
    const float force_y = (-m1 + m2 - m3 + m4) * sin_mount * thrust_per_motor_;

    // yaw torque: differential diagonal
    const float torque_z = (-m1 + m2 + m3 - m4) * arm_ * thrust_per_motor_;

    // speed in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef_water_;
    float speed_x = velocity_body.x;
    float speed_y = velocity_body.y;

    // hull drag on both axes (lateral drag 2x higher than forward)
    float drag_x = sq(speed_x) * hull_drag_coeff_;
    if (!is_positive(speed_x)) {
        drag_x *= -1.0f;
    }

    float drag_y = sq(speed_y) * hull_drag_coeff_ * 2.0f;
    if (!is_positive(speed_y)) {
        drag_y *= -1.0f;
    }

    // moment of inertia about z axis (simplified: I_zz = mass * arm^2)
    const float I_zz = mass_ * sq(arm_);

    // simplified damped yaw model: rate proportional to torque (rad/s)
    // (in reality torque/I_zz = angular accel, but we model heavy water damping)
    float yaw_rate = torque_z / I_zz;

    // set gyro
    gyro = Vector3f(0, 0, yaw_rate);

    // body frame acceleration
    accel_body.x = (force_x - drag_x) / mass_;
    accel_body.y = (force_y - drag_y) / mass_;
    accel_body.z = 0;
}

} // namespace SITL
