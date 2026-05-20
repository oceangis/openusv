// libraries/AR_OmniControl/AR_OmniControl.cpp
#include "AR_OmniControl.h"
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AR_OmniControl::var_info[] = {

    // @Param: _POS_P
    // @DisplayName: OMNIX position P gain
    // @Description: Proportional gain, NED position error (m) to force
    // @User: Standard
    AP_GROUPINFO("_POS_P", 1, AR_OmniControl, _pos_p, 0.20f),

    // @Param: _POS_I
    // @DisplayName: OMNIX position I gain
    // @User: Standard
    AP_GROUPINFO("_POS_I", 2, AR_OmniControl, _pos_i, 0.05f),

    // @Param: _POS_D
    // @DisplayName: OMNIX position D gain
    // @User: Standard
    AP_GROUPINFO("_POS_D", 3, AR_OmniControl, _pos_d, 0.0f),

    // @Param: _YAW_P
    // @DisplayName: OMNIX heading P gain
    // @User: Standard
    AP_GROUPINFO("_YAW_P", 4, AR_OmniControl, _yaw_p, 2.0f),

    // @Param: _YAW_I
    // @DisplayName: OMNIX heading I gain
    // @User: Standard
    AP_GROUPINFO("_YAW_I", 5, AR_OmniControl, _yaw_i, 0.10f),

    // @Param: _YAW_D
    // @DisplayName: OMNIX heading D gain
    // @User: Standard
    AP_GROUPINFO("_YAW_D", 6, AR_OmniControl, _yaw_d, 0.0f),

    // @Param: _POS_DB
    // @DisplayName: OMNIX position deadband
    // @Description: Position error below this is ignored (GPS noise suppression)
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("_POS_DB", 7, AR_OmniControl, _pos_db, 1.5f),

    // @Param: _YAW_DB
    // @DisplayName: OMNIX heading deadband
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("_YAW_DB", 8, AR_OmniControl, _yaw_db, 5.0f),

    // @Param: _SPEED
    // @DisplayName: OMNIX max correction speed
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("_SPEED", 9, AR_OmniControl, _speed_max, 1.0f),

    // @Param: _IMAX
    // @DisplayName: OMNIX integrator limit
    // @Description: Maximum integrator output for both position and yaw PID loops (normalized force/torque)
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("_IMAX", 10, AR_OmniControl, _imax, 0.5f),

    // @Param: _OPTIONS
    // @DisplayName: OMNIX options bitmask
    // @Description: reserved for stick-nudge and future options
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 11, AR_OmniControl, _options, 0),

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
