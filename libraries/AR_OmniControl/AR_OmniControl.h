// libraries/AR_OmniControl/AR_OmniControl.h
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

// Shared 3-DOF position + heading controller for OMNIX (X-config 4-thruster
// holonomic) boats. Used by ModeDP, ModeAuto (OMNIX branch), ModeGuided, etc.
// Owns PID parameters under prefix "DP_" (names preserved from original
// ModeDP for EEPROM key stability).
class AR_OmniControl {
public:
    AR_OmniControl();

    CLASS_NO_COPY(AR_OmniControl);

    // Set target NED position (m, from EKF origin), target heading (rad),
    // optional NED velocity feed-forward (m/s, reserved for path-follower
    // modes — unused in P1).
    void set_target(const Vector2f& pos_ned,
                    float yaw_rad,
                    const Vector2f& vel_ff_ned = Vector2f{0.0f, 0.0f});

    // Run PID + R(psi)^T rotation. Reads AHRS internally.
    // dt: loop period (s). Caller passes rover.G_Dt.
    void update(float dt);

    // Fetch most-recent outputs in body frame, normalized [-1, +1].
    // Returns false if AHRS position estimate was unavailable in last update;
    // caller should stop motors and trigger HOLD failsafe.
    bool get_outputs(float& fwd, float& lat, float& steer_norm) const;

    // Zero integrators and previous-error history. Call on mode entry.
    void reset();

    // Parameter table (DP_POS_P, DP_YAW_P, ... — see .cpp for full list).
    static const struct AP_Param::GroupInfo var_info[];

private:
    // --- parameters (registered in var_info) ---
    AP_Float _pos_p;
    AP_Float _pos_i;
    AP_Float _pos_d;
    AP_Float _yaw_p;
    AP_Float _yaw_i;
    AP_Float _yaw_d;
    AP_Float _pos_db;
    AP_Float _yaw_db;
    AP_Float _speed_max;
    AP_Float _imax;
    AP_Int16 _options;

    // --- target ---
    Vector2f _target_pos;
    float    _target_yaw;
    Vector2f _vel_ff;

    // --- PID state ---
    Vector2f _pos_integ;
    Vector2f _prev_pos_err;
    float    _yaw_integ;
    float    _prev_yaw_err;

    // --- last outputs ---
    float _out_fwd;
    float _out_lat;
    float _out_steer;
    bool  _outputs_valid;
};
