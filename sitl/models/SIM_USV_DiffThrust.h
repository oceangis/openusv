/*
  USV Differential Thrust simulator class
*/

#pragma once

#include "SIM_USV_Base.h"

namespace SITL {

/*
  a USV with differential thrust (skid-steering via two motors)
 */
class USV_DiffThrust : public USV_Base {
public:
    USV_DiffThrust(const char *frame_str);

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW USV_DiffThrust(frame_str);
    }

protected:
    void update_propulsion(const struct sitl_input &input, float delta_time) override;

private:
    static constexpr uint8_t USV_MOTOR_LEFT_CH = 0;    // servo1
    static constexpr uint8_t USV_MOTOR_RIGHT_CH = 2;   // servo3

    float motor_arm_;           // meters, distance from center to motor
    float skid_turn_rate_;      // deg/s at full differential
};

} // namespace SITL
