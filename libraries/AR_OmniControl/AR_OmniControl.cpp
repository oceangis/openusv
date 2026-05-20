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
    AP_AHRS& ahrs = AP::ahrs();

    // Position estimate required
    Vector3f pos_ned;
    if (!ahrs.get_relative_position_NED_origin_float(pos_ned)) {
        _out_fwd = _out_lat = _out_steer = 0.0f;
        _outputs_valid = false;
        return;
    }

    const float yaw = radians(ahrs.yaw_sensor * 0.01f);   // yaw_sensor in cdeg
    const Vector2f pos_now(pos_ned.x, pos_ned.y);

    // --- Position error + deadband ---
    Vector2f e = _target_pos - pos_now;
    if (e.length() < _pos_db) {
        e.zero();
        _pos_integ.zero();
    }

    // --- Position PID (NED frame vector) ---
    if (is_positive(dt)) {
        _pos_integ += e * (_pos_i * dt);
    }
    _pos_integ.x = constrain_float(_pos_integ.x, -_imax, _imax);
    _pos_integ.y = constrain_float(_pos_integ.y, -_imax, _imax);
    Vector2f pos_deriv;
    if (is_positive(dt)) {
        pos_deriv = (e - _prev_pos_err) * (1.0f / dt);
    }
    _prev_pos_err = e;
    const Vector2f f_ned = e * _pos_p + _pos_integ + pos_deriv * _pos_d;

    // --- Heading error + deadband ---
    float ey = wrap_PI(_target_yaw - yaw);
    if (fabsf(ey) < radians(_yaw_db)) {
        ey = 0.0f;
        _yaw_integ = 0.0f;
    }

    // --- Heading PID ---
    if (is_positive(dt)) {
        _yaw_integ += ey * (_yaw_i * dt);
    }
    _yaw_integ = constrain_float(_yaw_integ, -_imax, _imax);
    float yaw_deriv = 0.0f;
    if (is_positive(dt)) {
        yaw_deriv = (ey - _prev_yaw_err) * (1.0f / dt);
    }
    _prev_yaw_err = ey;
    float m_yaw = ey * _yaw_p + _yaw_integ + yaw_deriv * _yaw_d;

    // --- R(psi)^T: rotate NED force into body frame ---
    const float cy = cosf(yaw);
    const float sy = sinf(yaw);
    float forward =  cy * f_ned.x + sy * f_ned.y;
    float lateral = -sy * f_ned.x + cy * f_ned.y;

    // --- Saturate ---
    _out_fwd   = constrain_float(forward, -1.0f, 1.0f);
    _out_lat   = constrain_float(lateral, -1.0f, 1.0f);
    _out_steer = constrain_float(m_yaw,   -1.0f, 1.0f);
    _outputs_valid = true;
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
