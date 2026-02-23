/*
  USV Single Thruster + Rudder simulator class
*/

#pragma once

#include "SIM_USV_Base.h"

namespace SITL {

/*
  a USV with a single thruster and rudder steering (traditional boat)
 */
class USV_Rudder : public USV_Base {
public:
    USV_Rudder(const char *frame_str);

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW USV_Rudder(frame_str);
    }

protected:
    void update_propulsion(const struct sitl_input &input, float delta_time) override;

private:
    static constexpr uint8_t RUDDER_CH = 0;      // servo1
    static constexpr uint8_t THROTTLE_CH = 2;     // servo3
};

} // namespace SITL
