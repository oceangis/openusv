// libraries/AR_OmniControl/AR_OmniControl.cpp
#include "AR_OmniControl.h"
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AR_OmniControl::var_info[] = {
    // Populated in Task 2.
    AP_GROUPEND
};

AR_OmniControl::AR_OmniControl()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AR_OmniControl::set_target(const Vector2f& pos_ned, float yaw_rad,
                                const Vector2f& vel_ff_ned)
{
    _target_pos = pos_ned;
    _target_yaw = yaw_rad;
    _vel_ff = vel_ff_ned;
}

void AR_OmniControl::update(float dt)
{
    // Implemented in Task 4.
    (void)dt;
    _outputs_valid = false;
}

bool AR_OmniControl::get_outputs(float& fwd, float& lat, float& steer_norm) const
{
    fwd = _out_fwd;
    lat = _out_lat;
    steer_norm = _out_steer;
    return _outputs_valid;
}

void AR_OmniControl::reset()
{
    _pos_integ.zero();
    _prev_pos_err.zero();
    _yaw_integ = 0.0f;
    _prev_yaw_err = 0.0f;
    _out_fwd = _out_lat = _out_steer = 0.0f;
    _outputs_valid = false;
}
