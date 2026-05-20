#include "Rover.h"

/*
  ModeDP — Dynamic Positioning (X-config 4-thruster holonomic boats only).

  Holds GPS position and heading. PID/rotation kernel lives in
  AR_OmniControl (g2.omni_ctrl). This class only:
    - gates entry on FRAME_TYPE_OMNIX + position estimate
    - snapshots target on entry
    - dispatches update() to the shared controller
    - degrades to HOLD on lost position estimate

  Design: docs/superpowers/specs/2026-05-20-omnix-path-modes-design.md
*/

bool ModeDP::_enter()
{
    // Gate: OMNIX frame only
    if (g2.motors.get_frame_type() != AP_MotorsUGV::FRAME_TYPE_OMNIX) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DP: requires OMNIX frame");
        return false;
    }

    // Gate: need a position estimate to snapshot the target
    Vector3f pos_ned;
    if (!ahrs.get_relative_position_NED_origin_float(pos_ned)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DP: no position estimate");
        return false;
    }

    // Snapshot current position + heading as the hold target
    const Vector2f target_pos(pos_ned.x, pos_ned.y);
    const float target_yaw = radians(ahrs.yaw_sensor * 0.01f);
    g2.omni_ctrl.reset();
    g2.omni_ctrl.set_target(target_pos, target_yaw);
    _have_target = true;

    gcs().send_text(MAV_SEVERITY_INFO, "DP: holding position");
    return true;
}

void ModeDP::_exit()
{
    _have_target = false;
    g2.omni_ctrl.reset();
}

void ModeDP::update()
{
    if (!_have_target) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    g2.omni_ctrl.update(rover.G_Dt);

    float fwd, lat, steer_norm;
    if (!g2.omni_ctrl.get_outputs(fwd, lat, steer_norm)) {
        // Lost position estimate — stop motors, degrade to HOLD
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        rover.set_mode(rover.mode_hold, ModeReason::EKF_FAILSAFE);
        return;
    }

    g2.motors.set_throttle(fwd * 100.0f);
    g2.motors.set_lateral(lat * 100.0f);
    g2.motors.set_steering(steer_norm * 4500.0f, false);
}
