/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
    USV Wingsail Control - 翼帆控制模块 (统一框架)

    支持三种翼帆控制类型 (全部为刚性翼帆，无传统软帆):
    - WINGSAIL_ROTATION: 舵+翼帆旋转（控制整个翼帆角度）
    - WINGSAIL_FLAP: 舵+襟翼（控制翼帆上的襟翼，OpenTransat方式）
    - WINGSAIL_FREE: 只控舵（翼帆自平衡，Sailbuoy方式）

    统一框架架构:
    核心控制层(共享): 风向判断 -> 死区判断 -> 四条件控制
                     输出: 归一化 (-1.0 ~ +1.0)
    输出适配层: ROTATION(x ideal) | FLAP(x flap) | FREE(=0)
*/

#pragma once

// 翼帆类型枚举
enum class WingsailType : uint8_t {
    WINGSAIL_ROTATION = 0,  // 舵+翼帆旋转（控制整个翼帆角度）
    WINGSAIL_FLAP = 1,      // 舵+襟翼（控制翼帆上的襟翼）
    WINGSAIL_FREE = 2       // 只控舵（翼帆自平衡）
};

// 帆位置枚举 (左舷/右舷)
enum class SailSide : int8_t {
    PORT = -1,      // 左舷 (帆在左边，风从右边来)
    NEUTRAL = 0,    // 中立
    STARBOARD = 1   // 右舷 (帆在右边，风从左边来)
};

class Sailboat
{
public:

    // constructor
    Sailboat();

    // enabled
    bool sail_enabled() const { return enable > 0; }

    // true if sailboat navigation (aka tacking) is enabled
    bool tack_enabled() const;

    // setup
    void init();

    // initialise rc input (channel_wingsail)
    void init_rc_in();

    // calculate throttle and set wingsail
    void get_throttle_and_set_wingsail(float desired_speed, float &throttle_out);

    // Velocity Made Good
    float get_VMG() const;

    // handle user initiated tack while in acro mode
    void handle_tack_request_acro();

    // return target heading in radians when tacking (only used in acro)
    float get_tack_heading_rad();

    // handle user initiated tack while in autonomous modes
    void handle_tack_request_auto();

    // clear tacking state variables
    void clear_tack();

    // returns true if boat is currently tacking
    bool tacking() const;

    // returns true if sailboat should take indirect route to go upwind
    bool use_indirect_route(float desired_heading_cd) const;

    // calculate the heading to sail on if we can't go upwind
    float calc_heading(float desired_heading_cd);

    // states of USE_MOTOR parameter
    enum class UseMotor {
        USE_MOTOR_NEVER  = 0,
        USE_MOTOR_ASSIST = 1,
        USE_MOTOR_ALWAYS = 2
    };

    // set state of motor
    void set_motor_state(UseMotor state, bool report_failure = true);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // return sailboat loiter radius
    float get_loiter_radius() const { return loit_radius; }

    // set wingsail according to pilot input
    void set_pilot_desired_wingsail();

    // set wingsail in auto modes
    void set_auto_wingsail(float desired_speed);

    // relax wingsail (neutral position)
    void relax_sails();

    // ========== 统一翼帆控制接口 ==========

    // 获取翼帆类型
    WingsailType get_wingsail_type() const { return static_cast<WingsailType>(wingsail_type.get()); }

    // 获取当前归一化控制值 (-1.0 ~ +1.0)
    float get_normalized_control() const { return _normalized_control; }

    // 获取当前帆位置 (左舷/右舷)
    SailSide get_sail_side() const { return _sail_side; }

    // 获取当前帆角度 (相对于船头, 0~360度)
    float get_sail_angle() const { return _sail_angle_deg; }

    // 检测船是否在倒退
    bool detect_backward() const;

    // 获取倒退修正舵角 (-100~100), 0表示不需要修正
    float get_backward_correction_steering() const;

    // 计算有效死区角度 (考虑模式修正)
    float get_effective_no_go_angle() const;

    // 检查转向是否会穿过逆风点
    bool is_turn_against_wind(float wind_dir_rad, float from_heading_rad, float to_heading_rad) const;

    // ========== 差异化舵控制接口 ==========

    // 获取模式相关的舵增益系数
    // ROTATION: 0.8 (翼帆辅助转向，舵效减弱)
    // FLAP: 1.0 (标准舵效)
    // FREE: 1.3 (无翼帆辅助，需更大舵角)
    float get_steering_gain() const;

    // 获取统一舵修正值 (-100 ~ +100)
    // 包含: 倒退检测 + 模式增益 + 换舷辅助
    float get_steering_correction() const;

    // 获取换舷时的舵修正 (快速转向穿越逆风区)
    float get_tack_steering_correction() const;

private:

    // ========== 核心控制算法 (统一框架) ==========

    // 更新帆角度和位置 (所有模式共享)
    void update_sail_angle();

    // 核心控制算法: 计算归一化控制值 (-1.0 ~ +1.0)
    void calc_normalized_control();

    // 输出适配: 根据模式转换归一化值为实际输出
    float adapt_output_for_mode(float normalized) const;

    // 判断是否接近迎风 (用于增大控制量)
    bool is_near_upwind() const;

    // 风向确认检查 (防抖动)
    bool check_wind_confirmation(uint8_t index, bool condition);

    // ========== 电机辅助 ==========

    bool motor_assist_tack() const;
    bool motor_assist_low_wind() const;

    // ========== 参数定义 ==========
    AP_Int8 enable;                 // 启用帆船功能
    AP_Int8 wingsail_type;          // 翼帆类型 (WingsailType enum)

    // 翼帆旋转模式参数 (WINGSAIL_ROTATION)
    AP_Float sail_angle_ideal;      // 理想攻角 (默认25度)
    AP_Float sail_angle_max;        // 最大攻角 (迎风时, 默认40度)
    AP_Float sail_heel_angle_max;   // 最大倾斜角 (防倾覆, 默认15度)

    // 襟翼控制参数 (WINGSAIL_FLAP)
    AP_Int8 flap_max;               // 最大襟翼角度 (默认16度)
    AP_Int8 flap_normal;            // 正常襟翼角度 (默认10度)
    AP_Int8 flap_critical_angle;    // 迎风临界角度 (默认7度)

    // 通用航行参数
    AP_Float sail_no_go;            // 禁航区角度 (默认45度)
    AP_Float sail_windspeed_min;    // 最小风速 (低于此值使用电机)
    AP_Float xtrack_max;            // 最大横向偏差 (触发换舷)
    AP_Float loit_radius;           // 悬停半径

    // ========== 状态变量 ==========
    RC_Channel *channel_wingsail;   // RC输入通道
    bool currently_tacking;         // 正在换舷
    float tack_heading_rad;         // 换舷目标航向
    uint32_t tack_request_ms;       // 换舷请求时间
    uint32_t auto_tack_start_ms;    // 自动换舷开始时间
    uint32_t tack_clear_ms;         // 换舷清除时间
    bool tack_assist;               // 换舷辅助 (使用电机)
    UseMotor motor_state;           // 电机状态

    // ========== 统一控制状态 (核心) ==========
    float _sail_angle_deg;          // 帆角度 (0~360度, 0/360=迎风, 180=顺风)
    SailSide _sail_side;            // 帆位置 (左舷/右舷)
    float _normalized_control;      // 归一化控制值 (-1.0 ~ +1.0)
    bool _use_max_power;            // 是否使用最大控制量 (迎风时)
    uint32_t _control_update_ms;    // 控制更新时间戳
    uint32_t _wind_confirm_ms[4];   // 风向确认时间戳 (防抖)
};
