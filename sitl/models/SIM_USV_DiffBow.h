/*
  USV Differential Thrust + Bow Thruster simulator class
*/

#pragma once

#include "SIM_USV_DiffThrust.h"

namespace SITL {

/*
  a USV with differential thrust plus a lateral bow thruster

  Layout:
       ←→ S5 bow thruster (lateral)
      ┌──────────┐
      │  bow     │
  S1 ──┤          ├── S3
  left │  stern   │  right
      └──────────┘
 */
class USV_DiffBow : public USV_DiffThrust {
public:
    USV_DiffBow(const char *frame_str);

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW USV_DiffBow(frame_str);
    }

protected:
    void update_propulsion(const struct sitl_input &input, float delta_time) override;

private:
    static constexpr uint8_t BOW_THRUSTER_CH = 4;  // servo5, SERVO5_FUNCTION=33 (motor1, requires custom mixing)

    float bow_thrust_max_;      // Newtons
    float bow_arm_;             // meters from CG
};

} // namespace SITL
