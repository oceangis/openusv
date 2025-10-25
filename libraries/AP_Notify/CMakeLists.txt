# ArduPilot AP_Notify 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_BoardLED.cpp" "AP_BoardLED2.cpp" "AP_Notify.cpp" "Buzzer.cpp" "DiscreteRGBLed.cpp" "Display.cpp" "Display_SH1106_I2C.cpp" "Display_SITL.cpp" "Display_SSD1306_I2C.cpp" "DroneCAN_RGB_LED.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_BoardLED.cpp"
    "AP_BoardLED2.cpp"
    "AP_Notify.cpp"
    "Buzzer.cpp"
    "DiscreteRGBLed.cpp"
    "Display.cpp"
    "Display_SH1106_I2C.cpp"
    "Display_SITL.cpp"
    "Display_SSD1306_I2C.cpp"
    "DroneCAN_RGB_LED.cpp"
    "DShotLED.cpp"
    "ExternalLED.cpp"
    "GPIO_LED_1.cpp"
    "IS31FL3195.cpp"
    "LP5562.cpp"
    "MMLPlayer.cpp"
    "NavigatorLED.cpp"
    "NCP5623.cpp"
    "NeoPixel.cpp"
    "OreoLED_I2C.cpp"
    "PCA9685LED_I2C.cpp"
    "PixRacerLED.cpp"
    "ProfiLED.cpp"
    "RCOutputRGBLed.cpp"
    "RGBLed.cpp"
    "ScriptingLED.cpp"
    "SerialLED.cpp"
    "SITL_SFML_LED.cpp"
    "ToneAlarm.cpp"
    "ToshibaLED_I2C.cpp"
    "VRBoard_LED.cpp"
    "AP_BoardLED.cpp"
    "AP_BoardLED2.cpp"
    "AP_Notify.cpp"
    "Buzzer.cpp"
    "DiscreteRGBLed.cpp"
    "Display.cpp"
    "Display_SH1106_I2C.cpp"
    "Display_SITL.cpp"
    "Display_SSD1306_I2C.cpp"
    "DroneCAN_RGB_LED.cpp"
    "DShotLED.cpp"
    "ExternalLED.cpp"
    "GPIO_LED_1.cpp"
    "IS31FL3195.cpp"
    "LP5562.cpp"
    "MMLPlayer.cpp"
    "NavigatorLED.cpp"
    "NCP5623.cpp"
    "NeoPixel.cpp"
    "OreoLED_I2C.cpp"
    "PCA9685LED_I2C.cpp"
    "PixRacerLED.cpp"
    "ProfiLED.cpp"
    "RCOutputRGBLed.cpp"
    "RGBLed.cpp"
    "ScriptingLED.cpp"
    "SerialLED.cpp"
    "SITL_SFML_LED.cpp"
    "ToneAlarm.cpp"
    "ToshibaLED_I2C.cpp"
    "VRBoard_LED.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
