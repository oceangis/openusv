# ===========================================================================
# ArduPilot ESP32-IDF Pre-Build Automation
# 自动初始化 submodules 并生成必要的头文件
# ===========================================================================

function(ardupilot_prebuild)
    message(STATUS "==============================================")
    message(STATUS "ArduPilot Pre-Build Automation")
    message(STATUS "==============================================")

    # ---------------------------------------------------------------------------
    # 1. 检查 Git 可用性
    # ---------------------------------------------------------------------------
    find_package(Git REQUIRED)
    if(NOT GIT_FOUND)
        message(FATAL_ERROR "Git not found! Please install Git.")
    endif()

    # ---------------------------------------------------------------------------
    # 2. 检查 Python 可用性
    # ---------------------------------------------------------------------------
    if(DEFINED ENV{PYTHON})
        set(PYTHON_CMD "$ENV{PYTHON}")
    else()
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
        set(PYTHON_CMD "${Python3_EXECUTABLE}")
    endif()

    message(STATUS "Git: ${GIT_EXECUTABLE}")
    message(STATUS "Python: ${PYTHON_CMD}")

    # ---------------------------------------------------------------------------
    # 3. 初始化所有必要的 submodules
    # ---------------------------------------------------------------------------
    set(REQUIRED_SUBMODULES
        "mavlink"
        "DroneCAN/DSDL"
        "DroneCAN/dronecan_dsdlc"
        "DroneCAN/libcanard"
        "DroneCAN/pydronecan"
    )

    foreach(submod IN LISTS REQUIRED_SUBMODULES)
        set(submod_path "${CMAKE_SOURCE_DIR}/modules/${submod}")

        # 检查 submodule 是否已初始化
        if(NOT EXISTS "${submod_path}/.git")
            message(STATUS "Initializing submodule: ${submod}")
            execute_process(
                COMMAND "${GIT_EXECUTABLE}" submodule update --init --recursive "modules/${submod}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                RESULT_VARIABLE result
                OUTPUT_VARIABLE output
                ERROR_VARIABLE error
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_STRIP_TRAILING_WHITESPACE
            )

            if(NOT result EQUAL 0)
                message(FATAL_ERROR "Failed to initialize submodule ${submod}:\n${error}")
            endif()
            message(STATUS "  -> Initialized successfully")
        else()
            message(STATUS "Submodule ${submod}: OK")
        endif()
    endforeach()

    # ---------------------------------------------------------------------------
    # 4. 生成 DroneCAN 消息头文件
    # ---------------------------------------------------------------------------
    set(DRONECAN_DSDL_DIRS
        "${CMAKE_SOURCE_DIR}/modules/DroneCAN/DSDL/uavcan"
        "${CMAKE_SOURCE_DIR}/modules/DroneCAN/DSDL/dronecan"
        "${CMAKE_SOURCE_DIR}/modules/DroneCAN/DSDL/ardupilot"
        "${CMAKE_SOURCE_DIR}/modules/DroneCAN/DSDL/com"
        "${CMAKE_SOURCE_DIR}/modules/DroneCAN/DSDL/cuav"
        "${CMAKE_SOURCE_DIR}/modules/DroneCAN/DSDL/mppt"
    )

    set(DRONECAN_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/libraries/AP_DroneCAN")
    set(DRONECAN_DSDLC "${CMAKE_SOURCE_DIR}/modules/DroneCAN/dronecan_dsdlc/dronecan_dsdlc.py")
    set(DRONECAN_MARKER "${DRONECAN_OUTPUT_DIR}/include/.generated_stamp")

    # 检查是否需要重新生成
    set(need_generate FALSE)

    if(NOT EXISTS "${DRONECAN_MARKER}")
        set(need_generate TRUE)
        message(STATUS "DroneCAN headers not found, will generate")
    else()
        # 检查 DSDL 文件是否比生成的文件新
        file(GLOB_RECURSE DSDL_FILES
            "${CMAKE_SOURCE_DIR}/modules/DroneCAN/DSDL/*.uavcan"
        )

        foreach(dsdl_file IN LISTS DSDL_FILES)
            if("${dsdl_file}" IS_NEWER_THAN "${DRONECAN_MARKER}")
                set(need_generate TRUE)
                message(STATUS "DSDL files changed, will regenerate")
                break()
            endif()
        endforeach()
    endif()

    if(need_generate)
        message(STATUS "Generating DroneCAN message headers...")
        message(STATUS "  Output: ${DRONECAN_OUTPUT_DIR}")
        message(STATUS "  Script: ${DRONECAN_DSDLC}")

        # 创建输出目录
        file(MAKE_DIRECTORY "${DRONECAN_OUTPUT_DIR}/include")
        file(MAKE_DIRECTORY "${DRONECAN_OUTPUT_DIR}/src")

        # 构建 dronecan_dsdlc 命令
        set(dsdlc_cmd "${PYTHON_CMD}" "${DRONECAN_DSDLC}" "-O${DRONECAN_OUTPUT_DIR}")

        # 添加所有 DSDL 目录
        foreach(dsdl_dir IN LISTS DRONECAN_DSDL_DIRS)
            if(EXISTS "${dsdl_dir}")
                list(APPEND dsdlc_cmd "${dsdl_dir}")
                message(STATUS "  Adding DSDL: ${dsdl_dir}")
            endif()
        endforeach()

        # 执行生成
        execute_process(
            COMMAND ${dsdlc_cmd}
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            RESULT_VARIABLE result
            OUTPUT_VARIABLE output
            ERROR_VARIABLE error
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_STRIP_TRAILING_WHITESPACE
        )

        if(NOT result EQUAL 0)
            message(FATAL_ERROR "DroneCAN generation failed:\n${error}\n${output}")
        endif()

        # 创建标记文件
        string(TIMESTAMP current_time "%Y-%m-%d %H:%M:%S")
        file(WRITE "${DRONECAN_MARKER}"
             "Generated by: ${CMAKE_CURRENT_LIST_FILE}\nTimestamp: ${current_time}")

        message(STATUS "DroneCAN headers generated successfully")

        # 统计生成的文件
        file(GLOB_RECURSE GENERATED_HEADERS "${DRONECAN_OUTPUT_DIR}/include/*.h")
        file(GLOB_RECURSE GENERATED_SOURCES "${DRONECAN_OUTPUT_DIR}/src/*.c")
        list(LENGTH GENERATED_HEADERS num_headers)
        list(LENGTH GENERATED_SOURCES num_sources)
        message(STATUS "  Generated: ${num_headers} headers, ${num_sources} sources")
    else()
        message(STATUS "DroneCAN headers: up to date")
    endif()

    message(STATUS "==============================================")
    message(STATUS "Pre-Build Automation Complete")
    message(STATUS "==============================================")
endfunction()
