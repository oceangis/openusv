# ===========================================================================
# ArduPilot ESP32-IDF Pre-Build Automation
# ===========================================================================

function(ardupilot_prebuild)
    message(STATUS "==============================================")
    message(STATUS "ArduPilot Pre-Build Automation")
    message(STATUS "==============================================")

    # ---------------------------------------------------------------------------
    # 1. 读取板型配置 (从 board_config.cmake)
    # ---------------------------------------------------------------------------
    set(BOARD_CONFIG_FILE "${CMAKE_SOURCE_DIR}/board_config.cmake")
    if(EXISTS "${BOARD_CONFIG_FILE}")
        include("${BOARD_CONFIG_FILE}")
        message(STATUS "Board config loaded from: board_config.cmake")
    endif()

    # 如果没有设置，使用默认值
    if(NOT DEFINED ESP32_BOARD)
        set(ESP32_BOARD "esp32s3rover")
    endif()
    message(STATUS "Target board: ${ESP32_BOARD}")

    # ---------------------------------------------------------------------------
    # 2. 检查工具
    # ---------------------------------------------------------------------------
    find_package(Git QUIET)
    if(DEFINED ENV{PYTHON})
        set(PYTHON_CMD "$ENV{PYTHON}")
    else()
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
        set(PYTHON_CMD "${Python3_EXECUTABLE}")
    endif()

    # ---------------------------------------------------------------------------
    # 3. 初始化 submodules
    # ---------------------------------------------------------------------------
    set(REQUIRED_SUBMODULES "mavlink" "DroneCAN/DSDL" "DroneCAN/dronecan_dsdlc" "DroneCAN/libcanard")
    foreach(submod IN LISTS REQUIRED_SUBMODULES)
        set(submod_path "${CMAKE_SOURCE_DIR}/modules/${submod}")
        if(NOT EXISTS "${submod_path}/.git" AND GIT_FOUND)
            message(STATUS "Initializing: ${submod}")
            execute_process(
                COMMAND "${GIT_EXECUTABLE}" submodule update --init --recursive "modules/${submod}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            )
        endif()
    endforeach()

    # ---------------------------------------------------------------------------
    # 4. 生成 DroneCAN 头文件
    # ---------------------------------------------------------------------------
    set(DRONECAN_OUTPUT "${CMAKE_SOURCE_DIR}/libraries/AP_DroneCAN")
    set(DRONECAN_MARKER "${DRONECAN_OUTPUT}/include/.generated_stamp")
    set(DRONECAN_DSDLC "${CMAKE_SOURCE_DIR}/modules/DroneCAN/dronecan_dsdlc/dronecan_dsdlc.py")

    if(NOT EXISTS "${DRONECAN_MARKER}" AND EXISTS "${DRONECAN_DSDLC}")
        message(STATUS "Generating DroneCAN headers...")
        file(MAKE_DIRECTORY "${DRONECAN_OUTPUT}/include" "${DRONECAN_OUTPUT}/src")
        execute_process(
            COMMAND "${PYTHON_CMD}" "${DRONECAN_DSDLC}" "-O${DRONECAN_OUTPUT}"
                    "${CMAKE_SOURCE_DIR}/modules/DroneCAN/DSDL/uavcan"
                    "${CMAKE_SOURCE_DIR}/modules/DroneCAN/DSDL/dronecan"
                    "${CMAKE_SOURCE_DIR}/modules/DroneCAN/DSDL/ardupilot"
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        )
        file(WRITE "${DRONECAN_MARKER}" "Generated")
    endif()

    # ---------------------------------------------------------------------------
    # 5. 生成 hwdef.h (从板型目录的 hwdef.dat)
    # ---------------------------------------------------------------------------
    set(HWDEF_DIR "${CMAKE_SOURCE_DIR}/libraries/AP_HAL_ESP32/hwdef")
    set(HWDEF_DAT "${HWDEF_DIR}/${ESP32_BOARD}/hwdef.dat")
    set(HWDEF_H "${HWDEF_DIR}/hwdef.h")
    set(HWDEF_PY "${HWDEF_DIR}/scripts/esp32_hwdef.py")

    if(EXISTS "${HWDEF_DAT}" AND EXISTS "${HWDEF_PY}")
        # 检查是否需要重新生成
        set(need_gen FALSE)
        if(NOT EXISTS "${HWDEF_H}")
            set(need_gen TRUE)
        elseif("${HWDEF_DAT}" IS_NEWER_THAN "${HWDEF_H}")
            set(need_gen TRUE)
        endif()

        if(need_gen)
            message(STATUS "Generating hwdef.h from ${ESP32_BOARD}/hwdef.dat")
            execute_process(
                COMMAND "${PYTHON_CMD}" "${HWDEF_PY}" "--outdir" "${HWDEF_DIR}" "${HWDEF_DAT}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                RESULT_VARIABLE result
            )
            if(result EQUAL 0)
                message(STATUS "hwdef.h generated successfully")
            else()
                message(WARNING "hwdef.h generation failed, using existing file")
            endif()
        else()
            message(STATUS "hwdef.h: up to date")
        endif()
    else()
        message(STATUS "hwdef.h: using existing file")
    endif()

    message(STATUS "==============================================")
endfunction()
