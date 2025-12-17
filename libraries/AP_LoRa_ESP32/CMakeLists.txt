# AP_LoRa_ESP32 - LoRa MAVLink Parameter Integration

set(AP_LORA_ESP32_SRCS
    AP_LoRa_ESP32.cpp
)

set(AP_LORA_ESP32_HDRS
    AP_LoRa_ESP32.h
)

add_library(AP_LoRa_ESP32 STATIC ${AP_LORA_ESP32_SRCS})

target_include_directories(AP_LoRa_ESP32 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/components/lora_mavlink/include
)

target_link_libraries(AP_LoRa_ESP32 PUBLIC
    AP_Common
    AP_HAL
    AP_Param
)
