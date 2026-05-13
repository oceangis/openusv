# ===========================================================================
# ArduRover SITL Build — x86/Linux native compilation
# ===========================================================================
# Usage:
#   cmake -S sitl -B build_sitl -DCMAKE_BUILD_TYPE=RelWithDebInfo
#   cmake --build build_sitl -j$(nproc)
#
# Shares Rover/, libraries/, modules/DroneCAN/ with ESP-IDF build.
# Uses AP_HAL_SITL + libraries/SITL instead of AP_HAL_ESP32.
# ===========================================================================

cmake_minimum_required(VERSION 3.16)
project(ardurover_sitl CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(SITL_BUILD TRUE)

set(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")

# ===========================================================================
# Run PreBuild (DroneCAN header generation, submodule init)
# ===========================================================================
set(CMAKE_SOURCE_DIR_SAVED "${CMAKE_SOURCE_DIR}")
set(CMAKE_SOURCE_DIR "${PROJECT_ROOT}")
include("${PROJECT_ROOT}/cmake/PreBuild.cmake")
ardupilot_prebuild()
set(CMAKE_SOURCE_DIR "${CMAKE_SOURCE_DIR_SAVED}")

# ===========================================================================
# Collect source files
# ===========================================================================

# --- Libraries (all C/C++) ---
file(GLOB_RECURSE ALL_LIBRARY_SRCS
    "${PROJECT_ROOT}/libraries/*.c"
    "${PROJECT_ROOT}/libraries/*.cpp"
)

# --- Rover ---
file(GLOB_RECURSE ALL_ROVER_SRCS
    "${PROJECT_ROOT}/Rover/*.c"
    "${PROJECT_ROOT}/Rover/*.cpp"
)

# --- DroneCAN ---
file(GLOB LIBCANARD_SRCS
    "${PROJECT_ROOT}/modules/DroneCAN/libcanard/canard.c"
)
file(GLOB_RECURSE DRONECAN_GENERATED_SRCS
    "${PROJECT_ROOT}/libraries/AP_DroneCAN/src/*.c"
)

# --- Custom USV models ---
file(GLOB USV_MODEL_SRCS
    "${PROJECT_ROOT}/sitl/models/*.cpp"
    "${PROJECT_ROOT}/sitl/models/*.c"
)

set(ALL_SRCS ${ALL_LIBRARY_SRCS} ${ALL_ROVER_SRCS} ${LIBCANARD_SRCS} ${DRONECAN_GENERATED_SRCS} ${USV_MODEL_SRCS})

# ===========================================================================
# Exclude ESP32-specific and irrelevant files
# ===========================================================================

# Exclude ESP32 HAL (we use SITL HAL)
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_HAL_ESP32/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_WiFi_ESP32/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_LoRa_ESP32/.*\\.(c|cpp)$")

# Exclude AP_IOMCU (ChibiOS-only)
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_IOMCU/.*\\.(c|cpp)$")

# Exclude benchmarks, examples, tests
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/benchmarks/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/examples/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/tests/.*\\.(c|cpp)$")

# Keep: AP_HAL_SITL, AP_HAL_Empty, SITL (simulation framework)
# These are NOT excluded

# ===========================================================================
# Stage 1: Remove aircraft-specific libraries (same as ESP32 build)
# ===========================================================================
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_LandingGear/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_Parachute/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_Soaring/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_TECS/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_VideoTX/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_GyroFFT/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_Quicktune/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_Scripting/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_ADSB/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_Generator/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_Devo_Telem/.*\\.(c|cpp)$")
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_VisualOdom/.*\\.(c|cpp)$")

# ===========================================================================
# Stage 2: Remove sensors not needed for USV
# ===========================================================================
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AP_Beacon/.*\\.(c|cpp)$")

# ===========================================================================
# AC_* handling: exclude all, then re-add required
# ===========================================================================
list(FILTER ALL_SRCS EXCLUDE REGEX ".*/AC_[^/]+/.*\\.(c|cpp)$")

set(ROVER_REQUIRED_AC_MODULES
    AC_PosControl
    AC_PID
    AC_AttitudeControl
    AC_Fence
    AC_Avoidance
    AC_Sprayer
    AC_PrecLand
)
foreach(ac_module IN LISTS ROVER_REQUIRED_AC_MODULES)
    file(GLOB_RECURSE ac_module_srcs
        "${PROJECT_ROOT}/libraries/${ac_module}/*.c"
        "${PROJECT_ROOT}/libraries/${ac_module}/*.cpp"
    )
    list(APPEND ALL_SRCS ${ac_module_srcs})
endforeach()

# ===========================================================================
# Build the executable
# ===========================================================================
add_executable(ardurover ${ALL_SRCS})

# ===========================================================================
# Include directories
# ===========================================================================

# Auto-discover all library subdirectories
file(GLOB LIB_SUBDIRS LIST_DIRECTORIES true "${PROJECT_ROOT}/libraries/*/")
set(INCLUDE_DIRS "")
foreach(dir IN LISTS LIB_SUBDIRS)
    if(IS_DIRECTORY "${dir}")
        # Skip ESP32-specific directories
        get_filename_component(dirname "${dir}" NAME)
        if(dirname MATCHES "^(AP_HAL_ESP32|AP_WiFi_ESP32|AP_LoRa_ESP32)$")
            continue()
        endif()
        list(APPEND INCLUDE_DIRS "${dir}")
    endif()
endforeach()

target_include_directories(ardurover PRIVATE
    # Project directories
    "${PROJECT_ROOT}/Rover"
    "${PROJECT_ROOT}/libraries"
    "${PROJECT_ROOT}/libraries/AP_Common/missing"

    # SITL-specific
    "${PROJECT_ROOT}/libraries/AP_HAL_SITL"
    "${PROJECT_ROOT}/libraries/SITL"

    # Custom USV models (resolves "SIM_Aircraft.h" includes)
    "${PROJECT_ROOT}/sitl/models"

    # DroneCAN
    "${PROJECT_ROOT}/modules/DroneCAN/libcanard"
    "${PROJECT_ROOT}/libraries/AP_DroneCAN/include"

    # All library subdirectories
    ${INCLUDE_DIRS}
)

# ===========================================================================
# Compile definitions
# ===========================================================================
target_compile_definitions(ardurover PRIVATE
    CONFIG_HAL_BOARD=3                  # HAL_BOARD_SITL
    CONFIG_HAL_BOARD_SUBTYPE=0          # HAL_BOARD_SUBTYPE_NONE
    AP_SIM_ENABLED=1
    APM_BUILD_DIRECTORY=APM_BUILD_Rover
    __AP_LINE__=__LINE__

    # DroneCAN support
    USE_USER_HELPERS
    DRONECAN_CXX_WRAPPERS
    CANARD_ENABLE_DEADLINE=1
    CANARD_MULTI_IFACE=1
    CANARD_ALLOCATE_SEM=1

    # Disabled features (same as ESP32 build)
    HAL_AIRSPEED_ENABLED=0
    HAL_VISUALODOM_ENABLED=0
    HAL_PARACHUTE_ENABLED=0
    HAL_LANDING_GEAR_ENABLED=0
    HAL_SOARING_ENABLED=0
    AP_ADVANCEDFAILSAFE_ENABLED=0
    AP_VIDEOTX_ENABLED=0
    HAL_GYROFFT_ENABLED=0
    HAL_ADSB_ENABLED=0
    HAL_GENERATOR_ENABLED=0
    HAL_EFI_ENABLED=0
    HAL_OPTICALFLOW_ENABLED=0
    HAL_BEACON_ENABLED=0
)

# ===========================================================================
# Compile options
# ===========================================================================
target_compile_options(ardurover PRIVATE
    -Wno-format-security
    -Wno-unused-variable
    -Wno-unused-but-set-variable
    -Wno-maybe-uninitialized
    -Wno-trigraphs
)

# ===========================================================================
# Link libraries
# ===========================================================================
find_package(Threads REQUIRED)
target_link_libraries(ardurover PRIVATE Threads::Threads m rt dl)

# ===========================================================================
# Output binary location
# ===========================================================================
set_target_properties(ardurover PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
