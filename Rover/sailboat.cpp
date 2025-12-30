#include "Rover.h"

#define SAILBOAT_AUTO_TACKING_TIMEOUT_MS 5000
#define SAILBOAT_TACKING_ACCURACY_DEG 10
#define SAILBOAT_NOGO_PAD 10
#define TACK_RETRY_TIME_MS 5000
#define WIND_CONFIRMATION_MS 3000

const AP_Param::GroupInfo Sailboat::var_info[] = {

    AP_GROUPINFO_FLAGS("ENABLE", 1, Sailboat, enable, 0, AP_PARAM_FLAG_ENABLE),
    AP_GROUPINFO("ANGLE_IDEAL", 2, Sailboat, sail_angle_ideal, 25),
    AP_GROUPINFO("ANGLE_MAX", 3, Sailboat, sail_angle_max, 40),
    AP_GROUPINFO("HEEL_MAX", 4, Sailboat, sail_heel_angle_max, 15),
    AP_GROUPINFO("NO_GO_ANGLE", 5, Sailboat, sail_no_go, 45),
    AP_GROUPINFO("WNDSPD_MIN", 6, Sailboat, sail_windspeed_min, 0),
    AP_GROUPINFO("XTRACK_MAX", 7, Sailboat, xtrack_max, 10),
    AP_GROUPINFO("LOIT_RADIUS", 8, Sailboat, loit_radius, 5),
    AP_GROUPINFO("WING_TYPE", 9, Sailboat, wingsail_type, 0),
    AP_GROUPINFO("FLAP_MAX", 10, Sailboat, flap_max, 16),
    AP_GROUPINFO("FLAP_NORM", 11, Sailboat, flap_normal, 10),
    AP_GROUPINFO("FLAP_CRIT", 12, Sailboat, flap_critical_angle, 7),

    AP_GROUPEND
};

Sailboat::Sailboat() :
    _sail_angle_deg(0.0f),
    _sail_side(SailSide::NEUTRAL),
    _normalized_control(0.0f),
    _use_max_power(false),
    _control_update_ms(0),
    _wind_confirm_ms{0, 0, 0, 0}
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool Sailboat::tack_enabled() const
{
    if (!sail_enabled()) return false;
    if (motor_state == UseMotor::USE_MOTOR_ALWAYS) return false;
    if (motor_assist_low_wind()) return false;
    return true;
}

void Sailboat::init()
{
    if (sail_enabled()) {
#if AP_ROVER_CRASH_CHECK_ENABLED
        rover.g2.crash_angle.set_default(0);
#endif
        rover.g2.wp_nav.enable_overspeed(motor_state != UseMotor::USE_MOTOR_ALWAYS);
    }
    if (tack_enabled()) {
        rover.g2.loit_type.set_default(1);
    }
    set_motor_state(UseMotor::USE_MOTOR_ASSIST, false);

    _sail_angle_deg = 0.0f;
    _sail_side = SailSide::NEUTRAL;
    _normalized_control = 0.0f;
    _use_max_power = false;
    _control_update_ms = AP_HAL::millis();
    for (uint8_t i = 0; i < 4; i++) {
        _wind_confirm_ms[i] = AP_HAL::millis();
    }
}

void Sailboat::init_rc_in()
{
    RC_Channel *rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MAINSAIL);
    if (rc_ptr != nullptr) {
        channel_wingsail = rc_ptr;
        channel_wingsail->set_angle(100);
        channel_wingsail->set_default_dead_zone(30);
    } else {
        channel_wingsail = rover.channel_throttle;
    }
}

void Sailboat::set_pilot_desired_wingsail()
{
    if ((rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) || (channel_wingsail == nullptr)) {
       relax_sails();
    } else {
       rover.g2.motors.set_wingsail(constrain_float(channel_wingsail->get_control_in(), -100.0f, 100.0f));
    }
}

// ========== Core Control Algorithm (Unified Framework) ==========

void Sailboat::update_sail_angle()
{
    const float true_wind_rad = rover.g2.windvane.get_true_wind_direction_rad();
    const float heading_rad = rover.ahrs.get_yaw_rad();
    _sail_angle_deg = degrees(wrap_2PI(true_wind_rad - heading_rad));

    if (_sail_angle_deg < 180.0f) {
        _sail_side = SailSide::STARBOARD;
    } else {
        _sail_side = SailSide::PORT;
    }
}

bool Sailboat::is_near_upwind() const
{
    const float crit_angle = static_cast<float>(flap_critical_angle);
    return (_sail_angle_deg < crit_angle) || (_sail_angle_deg > (360.0f - crit_angle));
}

bool Sailboat::check_wind_confirmation(uint8_t index, bool condition)
{
    const uint32_t now = AP_HAL::millis();
    if (!condition) {
        _wind_confirm_ms[index] = now;
        return false;
    }
    return (now - _wind_confirm_ms[index]) >= WIND_CONFIRMATION_MS;
}

void Sailboat::calc_normalized_control()
{
    update_sail_angle();

    float error_deg = 0.0f;
    if (rover.control_mode->is_autopilot_mode()) {
        const float desired_heading_cd = rover.g2.wp_nav.wp_bearing_cd();
        const float desired_heading_rad = radians(desired_heading_cd * 0.01f);
        const float heading_rad = rover.ahrs.get_yaw_rad();
        error_deg = degrees(wrap_PI(desired_heading_rad - heading_rad));
    }

    const float crit_angle = static_cast<float>(flap_critical_angle);
    bool cond1 = (_sail_angle_deg < 180.0f && _normalized_control >= 0.0f);
    bool cond2 = (error_deg < 0.0f &&
                  (_sail_angle_deg > (360.0f - crit_angle) || _sail_angle_deg < crit_angle) &&
                  _normalized_control >= 0.0f);
    bool cond3 = (_sail_angle_deg > 180.0f && _normalized_control <= 0.0f);
    bool cond4 = (error_deg > 0.0f &&
                  (_sail_angle_deg < crit_angle || _sail_angle_deg > (360.0f - crit_angle)) &&
                  _normalized_control <= 0.0f);

    const float normal_power = 0.625f;
    const float max_power = 1.0f;

    if (check_wind_confirmation(0, cond1 || cond2)) {
        float new_control = cond2 ? -max_power : -normal_power;
        _use_max_power = cond2;
        if (fabsf(new_control - _normalized_control) > 0.01f) {
            _normalized_control = new_control;
            _control_update_ms = AP_HAL::millis();
        }
    }
    else if (check_wind_confirmation(1, cond3 || cond4)) {
        float new_control = cond4 ? max_power : normal_power;
        _use_max_power = cond4;
        if (fabsf(new_control - _normalized_control) > 0.01f) {
            _normalized_control = new_control;
            _control_update_ms = AP_HAL::millis();
        }
    }
    else if (check_wind_confirmation(2,
             _normalized_control <= -max_power + 0.01f &&
             _sail_angle_deg < 180.0f && _sail_angle_deg > 50.0f)) {
        _normalized_control = -normal_power;
        _use_max_power = false;
        _control_update_ms = AP_HAL::millis();
    }
    else if (check_wind_confirmation(3,
             _normalized_control >= max_power - 0.01f &&
             _sail_angle_deg > 180.0f && _sail_angle_deg < 310.0f)) {
        _normalized_control = normal_power;
        _use_max_power = false;
        _control_update_ms = AP_HAL::millis();
    }
}

float Sailboat::adapt_output_for_mode(float normalized) const
{
    const WingsailType wing_type = get_wingsail_type();

    switch (wing_type) {
        case WingsailType::WINGSAIL_ROTATION: {
            const float max_angle = _use_max_power ? sail_angle_max : sail_angle_ideal;
            float angle = normalized * max_angle;
            const float heel_reduction = rover.g2.attitude_control.get_sail_out_from_heel(
                radians(sail_heel_angle_max), rover.G_Dt);
            angle *= (1.0f - constrain_float(heel_reduction, 0.0f, 0.8f));
            return constrain_float(angle * 100.0f / sail_angle_max, -100.0f, 100.0f);
        }

        case WingsailType::WINGSAIL_FLAP: {
            const float max_flap = static_cast<float>(flap_max);
            const float flap_angle = normalized * max_flap;
            return constrain_float(flap_angle * 100.0f / max_flap, -100.0f, 100.0f);
        }

        case WingsailType::WINGSAIL_FREE:
        default:
            return 0.0f;
    }
}

void Sailboat::set_auto_wingsail(float desired_speed)
{
    calc_normalized_control();
    const float output = adapt_output_for_mode(_normalized_control);
    rover.g2.motors.set_wingsail(output);
}

void Sailboat::relax_sails()
{
    rover.g2.motors.set_wingsail(0.0f);
    _normalized_control = 0.0f;
    _sail_side = SailSide::NEUTRAL;
}

void Sailboat::get_throttle_and_set_wingsail(float desired_speed, float &throttle_out)
{
    throttle_out = 0.0f;
    if (!sail_enabled()) {
        relax_sails();
        return;
    }

    if ((motor_state == UseMotor::USE_MOTOR_ALWAYS) ||
         motor_assist_tack() || motor_assist_low_wind()) {
        throttle_out = 100.0f * rover.g2.attitude_control.get_throttle_out_speed(desired_speed,
                                                                        rover.g2.motors.limit.throttle_lower,
                                                                        rover.g2.motors.limit.throttle_upper,
                                                                        rover.g.speed_cruise,
                                                                        rover.g.throttle_cruise * 0.01f,
                                                                        rover.G_Dt);
    }

    if (motor_state == UseMotor::USE_MOTOR_ALWAYS) {
        relax_sails();
    } else {
        set_auto_wingsail(desired_speed);
    }
}

float Sailboat::get_VMG() const
{
    float speed;
    if (!rover.g2.attitude_control.get_forward_speed(speed)) {
        return 0.0f;
    }
    if (!rover.control_mode->is_autopilot_mode()) {
        return speed;
    }
    return (speed * cosf(wrap_PI(radians(rover.g2.wp_nav.wp_bearing_cd() * 0.01f) - rover.ahrs.get_yaw_rad())));
}

void Sailboat::handle_tack_request_acro()
{
    if (!tack_enabled() || currently_tacking) return;
    currently_tacking = true;
    tack_heading_rad = wrap_2PI(rover.ahrs.get_yaw_rad() + 2.0f * wrap_PI((rover.g2.windvane.get_true_wind_direction_rad() - rover.ahrs.get_yaw_rad())));
    tack_request_ms = AP_HAL::millis();
}

float Sailboat::get_tack_heading_rad()
{
    if (fabsf(wrap_PI(tack_heading_rad - rover.ahrs.get_yaw_rad())) < radians(SAILBOAT_TACKING_ACCURACY_DEG) ||
       ((AP_HAL::millis() - tack_request_ms) > SAILBOAT_AUTO_TACKING_TIMEOUT_MS)) {
        clear_tack();
    }
    return tack_heading_rad;
}

void Sailboat::handle_tack_request_auto()
{
    if (!tack_enabled() || currently_tacking) return;
    tack_request_ms = AP_HAL::millis();
}

void Sailboat::clear_tack()
{
    currently_tacking = false;
    tack_assist = false;
    tack_request_ms = 0;
    tack_clear_ms = AP_HAL::millis();
}

bool Sailboat::tacking() const
{
    return tack_enabled() && currently_tacking;
}

bool Sailboat::use_indirect_route(float desired_heading_cd) const
{
    if (!tack_enabled()) return false;
    if (currently_tacking) return true;

    const float desired_heading_rad = radians(desired_heading_cd * 0.01f);
    const float effective_no_go = get_effective_no_go_angle();
    return fabsf(wrap_PI(rover.g2.windvane.get_true_wind_direction_rad() - desired_heading_rad)) <= radians(effective_no_go + SAILBOAT_NOGO_PAD);
}

float Sailboat::calc_heading(float desired_heading_cd)
{
    if (!tack_enabled()) return desired_heading_cd;
    bool should_tack = false;

    const AP_WindVane::Sailboat_Tack current_tack = rover.g2.windvane.get_current_tack();
    const float desired_heading_rad = radians(desired_heading_cd * 0.01f);
    const float true_wind_rad = rover.g2.windvane.get_true_wind_direction_rad();
    const float current_heading_rad = rover.ahrs.get_yaw_rad();

    if (fabsf(wrap_PI(true_wind_rad - desired_heading_rad)) > radians(sail_no_go) && !currently_tacking) {
        const float new_heading_apparent_angle = wrap_PI(true_wind_rad - desired_heading_rad);
        AP_WindVane::Sailboat_Tack new_tack;
        if (is_negative(new_heading_apparent_angle)) {
            new_tack = AP_WindVane::Sailboat_Tack::TACK_PORT;
        } else {
            new_tack = AP_WindVane::Sailboat_Tack::TACK_STARBOARD;
        }

        if (new_tack != current_tack) {
            const float app_wind_rad = rover.g2.windvane.get_apparent_wind_direction_rad();
            if (fabsf(app_wind_rad) + fabsf(new_heading_apparent_angle) < M_PI) {
                is_turn_against_wind(true_wind_rad, current_heading_rad, desired_heading_rad);
                should_tack = true;
            }
        }
        if (!should_tack) return desired_heading_cd;
    }

    uint32_t now = AP_HAL::millis();
    if (tack_request_ms != 0 && !should_tack && !currently_tacking) {
        should_tack = ((now - tack_request_ms) < 500);
        tack_request_ms = 0;
    }

    const float cross_track_error = rover.g2.wp_nav.crosstrack_error();
    if ((fabsf(cross_track_error) >= xtrack_max) && !is_zero(xtrack_max) && !should_tack && !currently_tacking) {
        if (is_positive(cross_track_error) && (current_tack == AP_WindVane::Sailboat_Tack::TACK_STARBOARD)) {
            should_tack = true;
        }
        if (is_negative(cross_track_error) && (current_tack == AP_WindVane::Sailboat_Tack::TACK_PORT)) {
            should_tack = true;
        }
    }

    const float effective_no_go = get_effective_no_go_angle();
    const float left_no_go_heading_rad = wrap_2PI(true_wind_rad + radians(effective_no_go));
    const float right_no_go_heading_rad = wrap_2PI(true_wind_rad - radians(effective_no_go));

    if (should_tack && (now - tack_clear_ms) > TACK_RETRY_TIME_MS) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Sailboat: Tacking");
        switch (current_tack) {
            case AP_WindVane::Sailboat_Tack::TACK_PORT:
                tack_heading_rad = right_no_go_heading_rad;
                break;
            case AP_WindVane::Sailboat_Tack::TACK_STARBOARD:
                tack_heading_rad = left_no_go_heading_rad;
                break;
        }
        currently_tacking = true;
        auto_tack_start_ms = now;
    }

    if (currently_tacking) {
        if (fabsf(wrap_PI(tack_heading_rad - rover.ahrs.get_yaw_rad())) <= radians(SAILBOAT_TACKING_ACCURACY_DEG)) {
            clear_tack();
        } else if ((now - auto_tack_start_ms) > SAILBOAT_AUTO_TACKING_TIMEOUT_MS) {
            if ((motor_state == UseMotor::USE_MOTOR_ASSIST) && (now - auto_tack_start_ms) < (3.0f * SAILBOAT_AUTO_TACKING_TIMEOUT_MS)) {
                tack_assist = true;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Sailboat: Tacking timed out");
                clear_tack();
            }
        }
        return degrees(tack_heading_rad) * 100.0f;
    }

    if (current_tack == AP_WindVane::Sailboat_Tack::TACK_PORT) {
        return degrees(left_no_go_heading_rad) * 100.0f;
    } else {
        return degrees(right_no_go_heading_rad) * 100.0f;
    }
}

void Sailboat::set_motor_state(UseMotor state, bool report_failure)
{
    if (state == UseMotor::USE_MOTOR_NEVER) {
        motor_state = state;
        return;
    }

    if (rover.g2.motors.have_skid_steering() ||
        SRV_Channels::function_assigned(SRV_Channel::k_throttle) ||
        rover.get_frame_type() != rover.g2.motors.frame_type::FRAME_TYPE_UNDEFINED) {
        motor_state = state;
    } else if (report_failure) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Sailboat: failed to enable motor");
    }
}

bool Sailboat::motor_assist_tack() const
{
    if (motor_state != UseMotor::USE_MOTOR_ASSIST) return false;
    return tack_assist;
}

bool Sailboat::motor_assist_low_wind() const
{
    if (motor_state != UseMotor::USE_MOTOR_ASSIST) return false;
    return (is_positive(sail_windspeed_min) &&
            rover.g2.windvane.wind_speed_enabled() &&
            (rover.g2.windvane.get_true_wind_speed() < sail_windspeed_min));
}

// ========== Unified Control Helper Functions ==========

bool Sailboat::detect_backward() const
{
    return (_sail_angle_deg < 180.0f && _normalized_control > 0.0f) ||
           (_sail_angle_deg > 180.0f && _normalized_control < 0.0f);
}

bool Sailboat::is_turn_against_wind(float wind_dir_rad, float from_heading_rad, float to_heading_rad) const
{
    const float downwind_rad = wrap_2PI(wind_dir_rad + M_PI);
    const float angle_to_downwind = wrap_PI(downwind_rad - from_heading_rad);
    const float angle_to_target = wrap_PI(to_heading_rad - from_heading_rad);

    if (is_positive(angle_to_target)) {
        return (angle_to_downwind > 0.0f && angle_to_downwind < angle_to_target);
    } else {
        return (angle_to_downwind < 0.0f && angle_to_downwind > angle_to_target);
    }
}

float Sailboat::get_effective_no_go_angle() const
{
    const WingsailType wing_type = get_wingsail_type();

    switch (wing_type) {
        case WingsailType::WINGSAIL_FLAP: {
            const float flap_correction = static_cast<float>(flap_normal) - 3.0f;
            return constrain_float(sail_no_go - flap_correction, 30.0f, 60.0f);
        }
        case WingsailType::WINGSAIL_ROTATION: {
            const float rotation_correction = (sail_angle_max - sail_angle_ideal) * 0.2f;
            return constrain_float(sail_no_go - rotation_correction, 30.0f, 60.0f);
        }
        case WingsailType::WINGSAIL_FREE:
        default:
            return sail_no_go;
    }
}

float Sailboat::get_backward_correction_steering() const
{
    if (!detect_backward()) return 0.0f;

    const float upwind_zone = 45.0f + 2.0f * static_cast<float>(flap_normal);

    if (_sail_angle_deg > (180.0f - upwind_zone) && _sail_angle_deg < 180.0f && _normalized_control > 0.0f) {
        return 100.0f;
    }
    else if (_sail_angle_deg > 180.0f && _sail_angle_deg < (180.0f + upwind_zone) && _normalized_control < 0.0f) {
        return -100.0f;
    }

    return 0.0f;
}

// ========== Differential Steering Control ==========

float Sailboat::get_steering_gain() const
{
    const WingsailType wing_type = get_wingsail_type();

    switch (wing_type) {
        case WingsailType::WINGSAIL_ROTATION:
            // 翼帆旋转模式: 翼帆辅助转向，舵效减弱
            // 翼帆产生的侧向力辅助船转向，减小舵角需求
            return 0.8f;

        case WingsailType::WINGSAIL_FLAP:
            // 襟翼模式: 标准舵效
            // 襟翼仅改变翼帆升力，不直接影响转向
            return 1.0f;

        case WingsailType::WINGSAIL_FREE:
        default:
            // 自平衡模式: 无翼帆辅助，需更大舵角
            // 翼帆自由旋转，所有转向力由舵提供
            return 1.3f;
    }
}

float Sailboat::get_tack_steering_correction() const
{
    // 换舷时需要快速穿越逆风区，应用最大舵角
    if (!currently_tacking) {
        return 0.0f;
    }

    const float heading_error_rad = wrap_PI(tack_heading_rad - rover.ahrs.get_yaw_rad());
    const float heading_error_deg = degrees(heading_error_rad);

    // 换舷方向决定舵角方向
    // 正误差 = 需要右转 = 正舵角
    // 负误差 = 需要左转 = 负舵角
    const float tack_steering = constrain_float(heading_error_deg * 2.0f, -100.0f, 100.0f);

    // 应用模式增益
    return tack_steering * get_steering_gain();
}

float Sailboat::get_steering_correction() const
{
    // 优先级: 换舷 > 倒退修正 > 正常增益调整
    
    // 1. 换舷时使用换舷舵修正
    if (currently_tacking) {
        return get_tack_steering_correction();
    }

    // 2. 倒退检测修正
    const float backward_correction = get_backward_correction_steering();
    if (!is_zero(backward_correction)) {
        // 倒退时应用增益调整
        return backward_correction * get_steering_gain();
    }

    // 3. 正常情况下返回0，由mode.cpp应用增益
    // get_steering_gain() 在 mode.cpp 中应用于最终舵输出
    return 0.0f;
}
