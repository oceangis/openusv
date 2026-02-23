/*
  USV Wingsail simulator class
  Handles both rotation-only wingsail and wingsail+flap variants
*/

#pragma once

#include "SIM_USV_Base.h"

namespace SITL {

/*
  a USV with wingsail propulsion (rotation only, or rotation + trailing-edge flap)
 */
class USV_Wingsail : public USV_Base {
public:
    USV_Wingsail(const char *frame_str);

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW USV_Wingsail(frame_str);
    }

protected:
    void update_propulsion(const struct sitl_input &input, float delta_time) override;

private:
    static constexpr uint8_t RUDDER_CH = 0;      // servo1
    static constexpr uint8_t WINGSAIL_CH = 2;     // servo3
    static constexpr uint8_t FLAP_CH = 3;         // servo4

    bool has_flap_;             // detected from frame string
    float sail_area_;           // m^2

    // calculate lift and drag given wind speed (m/s) and angle-of-attack (deg)
    void calc_lift_and_drag(float wind_speed, float angle_of_attack_deg, float &lift, float &drag) const;

    // aerodynamic lookup tables (18 points at 10 deg intervals, 0-170 deg)
    static const float lift_curve_[18];
    static const float drag_curve_[18];
};

} // namespace SITL
