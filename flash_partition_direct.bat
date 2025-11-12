@echo off
REM 直接使用esptool.py烧录分区表
setlocal

set PYTHON=D:\Espressif\tools\python_env\idf5.5_py3.11_env\Scripts\python.exe
set ESPTOOL=D:\Espressif\v5.5.1\esp-idf\components\esptool_py\esptool\esptool.py
set COM_PORT=COM3
set PARTITION_BIN=build\partition_table\partition-table.bin

echo ========================================
echo 使用esptool.py烧录分区表
echo ========================================
echo.
echo 串口: %COM_PORT%
echo 分区表文件: %PARTITION_BIN%
echo.

REM 检查分区表文件是否存在
if not exist %PARTITION_BIN% (
    echo 错误：分区表文件不存在！
    echo 请先运行: build.bat build
    goto :end
)

echo 正在烧录分区表...
echo.

%PYTHON% %ESPTOOL% --chip esp32s3 --port %COM_PORT% --baud 460800 write_flash 0x8000 %PARTITION_BIN%

if errorlevel 1 (
    echo.
    echo ========================================
    echo 烧录失败！
    echo ========================================
    echo.
    echo 可能的原因：
    echo 1. ESP32-S3未连接到%COM_PORT%
    echo 2. 设备未进入下载模式（尝试按住BOOT键后按RST键）
    echo 3. 串口被其他程序占用
    echo 4. 串口驱动未安装
    echo.
    goto :end
)

echo.
echo ========================================
echo 烧录成功！
echo ========================================
echo.
echo 接下来请：
echo 1. 按下ESP32-S3的RST（复位）按键
echo 2. 打开串口监视器查看启动日志
echo 3. 检查是否出现: "Storage partition found"
echo.

:end
endlocal
pause
