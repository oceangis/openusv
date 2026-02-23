/*
  USV Base simulator class - common water physics for all USV boat types
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  base class for all USV (Unmanned Surface Vehicle) simulator types
 */
class USV_Base : public Aircraft {
public:
    USV_Base(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    bool on_ground() const override { return true; }

protected:
    // derived classes must implement this to set accel_body and gyro
    // from servo inputs. delta_time is in seconds.
    virtual void update_propulsion(const struct sitl_input &input, float delta_time) = 0;

    // vehicle parameters (set by derived classes in constructor)
    float mass_;
    float steering_angle_max_;      // degrees
    float turning_circle_;          // meters (diameter)
    float max_thrust_;              // Newtons
    float hull_drag_coeff_;         // drag coefficient

    // water velocity (excludes tide)
    Vector3f velocity_ef_water_;

    // wave state
    Vector3f wave_gyro_;            // rad/s
    float wave_heave_;              // m/s/s
    float wave_phase_;              // rads

    // return yaw rate in deg/sec given steering input (-1 to +1) and speed in m/s
    float get_yaw_rate_from_steering(float steering, float speed) const;

    // return turning circle (diameter) in meters for steering proportion (-1 to +1)
    float get_turn_circle(float steering) const;

private:
    // simulate basic waves / swell
    void update_wave(float delta_time);

    // integrate velocity and position from accel_body
    void integrate_state(float delta_time);

};

} // namespace SITL
