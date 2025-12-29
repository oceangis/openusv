#pragma once

#include "defines.h"

#ifndef MAV_SYSTEM_ID
  #define MAV_SYSTEM_ID    1
#endif

//////////////////////////////////////////////////////////////////////////////
// FrSky telemetry support
//

#ifndef CH7_OPTION
  #define CH7_OPTION CH7_SAVE_WP
#endif

//////////////////////////////////////////////////////////////////////////////
// MODE
// MODE_CHANNEL
//
#ifndef MODE_CHANNEL
  #define MODE_CHANNEL    8
#endif
#if (MODE_CHANNEL != 5) && (MODE_CHANNEL != 6) && (MODE_CHANNEL != 7) && (MODE_CHANNEL != 8)
  #error XXX
  #error XXX You must set MODE_CHANNEL to 5, 6, 7 or 8
  #error XXX
#endif

//////////////////////////////////////////////////////////////////////////////
// NAVL1
//
#ifndef NAVL1
  #define NAVL1_PERIOD    8
#endif

//////////////////////////////////////////////////////////////////////////////
// CRUISE_SPEED default
//
#ifndef CRUISE_SPEED
  #define CRUISE_SPEED    2  // in m/s
#endif

#define DEFAULT_LOG_BITMASK    0xffff

//////////////////////////////////////////////////////////////////////////////
// Mode enable/disable - default to enabled if not defined in hwdef.dat
//////////////////////////////////////////////////////////////////////////////
#ifndef MODE_ACRO_ENABLED
# define MODE_ACRO_ENABLED 1
#endif

#ifndef MODE_STEERING_ENABLED
# define MODE_STEERING_ENABLED 1
#endif

#ifndef MODE_CIRCLE_ENABLED
# define MODE_CIRCLE_ENABLED 1
#endif

#ifndef MODE_SMARTRTL_ENABLED
# define MODE_SMARTRTL_ENABLED 1
#endif

#ifndef MODE_SIMPLE_ENABLED
# define MODE_SIMPLE_ENABLED 1
#endif

#ifndef MODE_POSHOLD_ENABLED
# define MODE_POSHOLD_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// Dock mode - allows vehicle to dock to a docking target
#ifndef MODE_DOCK_ENABLED
# define MODE_DOCK_ENABLED AC_PRECLAND_ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Follow mode - allows vehicle to follow target
#ifndef MODE_FOLLOW_ENABLED
# define MODE_FOLLOW_ENABLED AP_FOLLOW_ENABLED
#endif


//////////////////////////////////////////////////////////////////////////////
// Developer Items
//

// if RESET_SWITCH_CH is not zero, then this is the PWM value on
// that channel where we reset the control mode to the current switch
// position (to for example return to switched mode after failsafe or
// fence breach)
#ifndef RESET_SWITCH_CHAN_PWM
  #define RESET_SWITCH_CHAN_PWM    1750
#endif

#ifndef AP_ROVER_ADVANCED_FAILSAFE_ENABLED
  #define AP_ROVER_ADVANCED_FAILSAFE_ENABLED 0
#endif

#ifndef AP_ROVER_AUTO_ARM_ONCE_ENABLED
#define AP_ROVER_AUTO_ARM_ONCE_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// USV (Unmanned Surface Vehicle) 专用配置
// 禁用陆地车辆特有功能以减少代码体积和内存占用
//////////////////////////////////////////////////////////////////////////////

// 禁用轮编码器 - USV无轮子
#ifndef AP_WHEELENCODER_ENABLED
#define AP_WHEELENCODER_ENABLED 0
#endif

// 禁用轮速率控制 - USV无轮子
#ifndef AP_WHEELRATECONTROL_ENABLED
#define AP_WHEELRATECONTROL_ENABLED 0
#endif

// 禁用碰撞/翻倒检测 - USV不会像陆地车辆那样翻倒
#ifndef AP_ROVER_CRASH_CHECK_ENABLED
#define AP_ROVER_CRASH_CHECK_ENABLED 0
#endif

// 禁用 MSP 协议 - USV不需要 Betaflight OSD
#ifndef HAL_MSP_ENABLED
#define HAL_MSP_ENABLED 0
#endif

// 禁用室内定位信标 - USV在户外使用GPS
#ifndef AP_BEACON_ENABLED
#define AP_BEACON_ENABLED 0
#endif

// 禁用农药喷洒器 - USV不需要
#ifndef HAL_SPRAYER_ENABLED
#define HAL_SPRAYER_ENABLED 0
#endif

// 禁用精确着陆 - 飞行器功能
#ifndef AC_PRECLAND_ENABLED
#define AC_PRECLAND_ENABLED 0
#endif

// 禁用对接模式 - 依赖精确着陆
#ifndef MODE_DOCK_ENABLED
#define MODE_DOCK_ENABLED 0
#endif

// 禁用光流传感器 - 飞行器悬停用
#ifndef AP_OPTICALFLOW_ENABLED
#define AP_OPTICALFLOW_ENABLED 0
#endif

// 禁用空速计 - 飞行器用
#ifndef AP_AIRSPEED_ENABLED
#define AP_AIRSPEED_ENABLED 0
#endif
