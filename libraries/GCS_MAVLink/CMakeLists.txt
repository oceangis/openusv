# ArduPilot GCS_MAVLink 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "GCS.cpp" "GCS_Common.cpp" "GCS_DeviceOp.cpp" "GCS_Dummy.cpp" "GCS_Fence.cpp" "GCS_FTP.cpp" "GCS_MAVLink.cpp" "GCS_MAVLink_Parameters.cpp" "GCS_Param.cpp" "GCS_Rally.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "GCS.cpp"
    "GCS_Common.cpp"
    "GCS_DeviceOp.cpp"
    "GCS_Dummy.cpp"
    "GCS_Fence.cpp"
    "GCS_FTP.cpp"
    "GCS_MAVLink.cpp"
    "GCS_MAVLink_Parameters.cpp"
    "GCS_Param.cpp"
    "GCS_Rally.cpp"
    "GCS_serial_control.cpp"
    "GCS_ServoRelay.cpp"
    "GCS_Signing.cpp"
    "MAVLink_routing.cpp"
    "MissionItemProtocol.cpp"
    "MissionItemProtocol_Fence.cpp"
    "MissionItemProtocol_Rally.cpp"
    "MissionItemProtocol_Waypoints.cpp"
    "GCS.cpp"
    "GCS_Common.cpp"
    "GCS_DeviceOp.cpp"
    "GCS_Dummy.cpp"
    "GCS_Fence.cpp"
    "GCS_FTP.cpp"
    "GCS_MAVLink.cpp"
    "GCS_MAVLink_Parameters.cpp"
    "GCS_Param.cpp"
    "GCS_Rally.cpp"
    "GCS_serial_control.cpp"
    "GCS_ServoRelay.cpp"
    "GCS_Signing.cpp"
    "MAVLink_routing.cpp"
    "MissionItemProtocol.cpp"
    "MissionItemProtocol_Fence.cpp"
    "MissionItemProtocol_Rally.cpp"
    "MissionItemProtocol_Waypoints.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)

# Generate MAVLink headers (ESP-IDF prebuild)
# Always use in-repo submodule at modules/mavlink; do not rely on external paths

# 1) Ensure submodule is initialised/updated (once) via a stamp file
set(_mavlink_dir "${CMAKE_SOURCE_DIR}/modules/mavlink")
set(_mavlink_stamp "${_mavlink_dir}/.stamp_init")

add_custom_command(
    OUTPUT "${_mavlink_stamp}"
    COMMAND ${CMAKE_COMMAND} -E echo "Updating git submodule: modules/mavlink"
    COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_SOURCE_DIR}" git submodule update --init --recursive modules/mavlink
    COMMAND ${CMAKE_COMMAND} -E touch "${_mavlink_stamp}"
    COMMENT "Initializing/Updating MAVLink submodule"
    VERBATIM)

add_custom_target(mavlink_submodule_init DEPENDS "${_mavlink_stamp}")

# 2) Generate headers with pymavlink's mavgen.py from the submodule
set(_mavgen_py "${_mavlink_dir}/pymavlink/tools/mavgen.py")
set(_mavxml    "${_mavlink_dir}/message_definitions/v1.0/all.xml")
set(_mavout    "${CMAKE_CURRENT_LIST_DIR}/include/mavlink/v2.0")

# Prefer using IDF-provided Python if available; fallback to system python
set(_python_cmd "python")
if(DEFINED ENV{PYTHON})
    set(_python_cmd "$ENV{PYTHON}")
endif()

add_custom_command(
    OUTPUT "${_mavout}/all/version.h"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_mavout}"
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${_mavlink_dir} ${_python_cmd} "${_mavgen_py}" --lang C --wire-protocol 2.0 --output "${_mavout}" "${_mavxml}"
    DEPENDS mavlink_submodule_init "${_mavgen_py}" "${_mavxml}"
    COMMENT "Generating MAVLink headers into ${_mavout}"
    VERBATIM)

add_custom_target(generate_mavlink ALL DEPENDS "${_mavout}/all/version.h")
add_dependencies(${COMPONENT_LIB} generate_mavlink)

# Relax trigraph warnings for IDF toolchain
target_compile_options(${COMPONENT_LIB} PRIVATE
    -fsingle-precision-constant
    -Wno-error=trigraphs
    -Wno-trigraphs
)
