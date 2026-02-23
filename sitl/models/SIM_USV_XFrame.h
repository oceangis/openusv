/*
  USV X-Frame quad thruster simulator class
*/

#pragma once

#include "SIM_USV_Base.h"

namespace SITL {

/*
  a USV with 4 motors in X configuration for omnidirectional movement
 */
class USV_XFrame : public USV_Base {
public:
    USV_XFrame(const char *frame_str);

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW USV_XFrame(frame_str);
    }

protected:
    void update_propulsion(const struct sitl_input &input, float delta_time) override;

private:
    static constexpr uint8_t MOTOR_FL_CH = 0;     // servo1 - front left
    static constexpr uint8_t MOTOR_FR_CH = 1;     // servo2 - front right
    static constexpr uint8_t MOTOR_BL_CH = 2;     // servo3 - back left
    static constexpr uint8_t MOTOR_BR_CH = 3;     // servo4 - back right

    float arm_;                 // meters, distance from CG to motor
    float mount_angle_;         // degrees, motor mount angle from centerline
    float thrust_per_motor_;    // Newtons, max thrust per motor
};

} // namespace SITL
