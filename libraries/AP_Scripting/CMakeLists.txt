# ArduPilot AP_Scripting 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "lua_generated_bindings.cpp" "AP_Scripting.cpp" "AP_Scripting_CANSensor.cpp" "AP_Scripting_CRSFMenu.cpp" "AP_Scripting_helpers.cpp" "AP_Scripting_SerialAccess.cpp" "AP_Scripting_SerialDevice.cpp" "lua_bindings.cpp" "lua_boxed_numerics.cpp" "lua_scripts.cpp" "AP_Scripting.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "applets" "applets/Aerobatics" "applets/Aerobatics/FixedWing" "applets/Aerobatics/FixedWing/RateBased"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Scripting.cpp"
    "AP_Scripting_CANSensor.cpp"
    "AP_Scripting_CRSFMenu.cpp"
    "AP_Scripting_helpers.cpp"
    "AP_Scripting_SerialAccess.cpp"
    "AP_Scripting_SerialDevice.cpp"
    "lua_bindings.cpp"
    "lua_boxed_numerics.cpp"
    "lua_scripts.cpp"
    "AP_Scripting.cpp"
    "AP_Scripting_CANSensor.cpp"
    "AP_Scripting_CRSFMenu.cpp"
    "AP_Scripting_helpers.cpp"
    "AP_Scripting_SerialAccess.cpp"
    "AP_Scripting_SerialDevice.cpp"
    "lua_bindings.cpp"
    "lua_boxed_numerics.cpp"
    "lua_scripts.cpp"
    "lua/src/lapi.c"
    "lua/src/lauxlib.c"
    "lua/src/lbaselib.c"
    "lua/src/lbitlib.c"
    "lua/src/lcode.c"
    "lua/src/lcorolib.c"
    "lua/src/lctype.c"
    "lua/src/ldblib.c"
    "lua/src/ldebug.c"
    "lua/src/ldo.c"
    "lua/src/ldump.c"
    "lua/src/lfunc.c"
    "lua/src/lgc.c"
    "lua/src/linit.c"
    "lua/src/liolib.c"
    "lua/src/llex.c"
    "lua/src/lmathlib.c"
    "lua/src/lmem.c"
    "lua/src/loadlib.c"
    "lua/src/lobject.c"
    "lua/src/lopcodes.c"
    "lua/src/loslib.c"
    "lua/src/lparser.c"
    "lua/src/lstate.c"
    "lua/src/lstring.c"
    "lua/src/lstrlib.c"
    "lua/src/ltable.c"
    "lua/src/ltablib.c"
    "lua/src/ltm.c"
    "lua/src/lundump.c"
    "lua/src/lutf8lib.c"
    "lua/src/lvm.c"
    "lua/src/lzio.c"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "applets"
    "applets/Aerobatics"
    "applets/Aerobatics/FixedWing"
    "applets/Aerobatics/FixedWing/RateBased"
    "applets/Aerobatics/FixedWing/Schedules"
    "applets/WebExamples"
    "docs"
    "drivers"
    "drivers/TOFSense-M"
    "lua"
    "lua/doc"
    "lua/src"
    "modules"
    "modules/MAVLink"
)
