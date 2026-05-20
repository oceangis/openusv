#include "Rover.h"

/*
  ModeDP — 动力定位 (Dynamic Positioning)

  仅用于 X 型 4 推进器全驱动 (holonomic) 船。同时锁定 GPS 位置和艏向。
  Fossen 3-DOF PID:NED 位置/航向误差 → PID → NED 系力 → R(psi)^T 旋到
  船体系 → forward / lateral / yaw → AP_MotorsUGV::output_omni() 4 电机混控。

  设计文档:docs/superpowers/specs/2026-05-17-omnix-dp-mode-design.md
*/

const AP_Param::GroupInfo ModeDP::var_info[] = {

    // @Param: _POS_P
    // @DisplayName: DP position P gain
    // @Description: Proportional gain, NED position error (m) to force
    // @User: Standard
    AP_GROUPINFO("_POS_P", 1, ModeDP, _pos_p, 0.20f),

    // @Param: _POS_I
    // @DisplayName: DP position I gain
    // @User: Standard
    AP_GROUPINFO("_POS_I", 2, ModeDP, _pos_i, 0.05f),

    // @Param: _POS_D
    // @DisplayName: DP position D gain
    // @User: Standard
    AP_GROUPINFO("_POS_D", 3, ModeDP, _pos_d, 0.0f),

    // @Param: _YAW_P
    // @DisplayName: DP heading P gain
    // @User: Standard
    AP_GROUPINFO("_YAW_P", 4, ModeDP, _yaw_p, 2.0f),

    // @Param: _YAW_I
    // @DisplayName: DP heading I gain
    // @User: Standard
    AP_GROUPINFO("_YAW_I", 5, ModeDP, _yaw_i, 0.10f),

    // @Param: _YAW_D
    // @DisplayName: DP heading D gain
    // @User: Standard
    AP_GROUPINFO("_YAW_D", 6, ModeDP, _yaw_d, 0.0f),

    // @Param: _POS_DB
    // @DisplayName: DP position deadband
    // @Description: Position error below this is ignored (suppress GPS noise)
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("_POS_DB", 7, ModeDP, _pos_db, 1.5f),

    // @Param: _YAW_DB
    // @DisplayName: DP heading deadband
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("_YAW_DB", 8, ModeDP, _yaw_db, 5.0f),

    // @Param: _SPEED
    // @DisplayName: DP max correction speed
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("_SPEED", 9, ModeDP, _speed_max, 1.0f),

    // @Param: _IMAX
    // @DisplayName: DP integrator limit
    // @User: Standard
    AP_GROUPINFO("_IMAX", 10, ModeDP, _imax, 0.5f),

    // @Param: _OPTIONS
    // @DisplayName: DP options bitmask
    // @Description: reserved for stick-nudge and future options
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 11, ModeDP, _options, 0),

    AP_GROUPEND
};

ModeDP::ModeDP() : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeDP::_enter()
{
    // 门禁:DP 仅对 X 型 4 推进器全驱动船开放
    if (g2.motors.get_frame_type() != AP_MotorsUGV::FRAME_TYPE_OMNIX) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DP: requires OMNIX frame");
        return false;
    }

    // 抓取当前位置 + 艏向作为 DP 目标
    Vector3f pos_ned;
    if (!ahrs.get_relative_position_NED_origin_float(pos_ned)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DP: no position estimate");
        return false;
    }
    _target_pos = Vector2f(pos_ned.x, pos_ned.y);
    _target_yaw = radians(ahrs.yaw_sensor * 0.01f);   // yaw_sensor 单位:厘度

    _pos_integ.zero();
    _yaw_integ = 0.0f;
    _prev_pos_err.zero();
    _prev_yaw_err = 0.0f;
    _have_target = true;

    gcs().send_text(MAV_SEVERITY_INFO, "DP: holding position");
    return true;
}

void ModeDP::_exit()
{
    _have_target = false;
}

void ModeDP::update()
{
    // 失效保护:丢位置估计 → 停桨
    Vector3f pos_ned;
    if (!_have_target ||
        !ahrs.get_relative_position_NED_origin_float(pos_ned)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    const float dt = rover.G_Dt;
    const float yaw = radians(ahrs.yaw_sensor * 0.01f);   // yaw_sensor 单位:厘度
    const Vector2f pos_now(pos_ned.x, pos_ned.y);

    // --- 位置误差 + 死区(波浪滤波 lite)---
    Vector2f e = _target_pos - pos_now;
    if (e.length() < _pos_db) {
        e.zero();
        _pos_integ.zero();
    }

    // --- 位置环 PID(NED 系矢量)---
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

    // --- 航向误差 + 死区 ---
    float ey = wrap_PI(_target_yaw - yaw);
    if (fabsf(ey) < radians(_yaw_db)) {
        ey = 0.0f;
        _yaw_integ = 0.0f;
    }

    // --- 航向环 PID ---
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

    // --- R(psi)^T:把 NED 系力旋到船体系(DP 控制律核心)---
    const float cy = cosf(yaw);
    const float sy = sinf(yaw);
    float forward =  cy * f_ned.x + sy * f_ned.y;
    float lateral = -sy * f_ned.x + cy * f_ned.y;

    // --- 限幅 + 输出 ---
    forward = constrain_float(forward, -1.0f, 1.0f);
    lateral = constrain_float(lateral, -1.0f, 1.0f);
    m_yaw   = constrain_float(m_yaw,   -1.0f, 1.0f);

    g2.motors.set_throttle(forward * 100.0f);
    g2.motors.set_lateral(lateral * 100.0f);
    g2.motors.set_steering(m_yaw * 4500.0f, false);
}
