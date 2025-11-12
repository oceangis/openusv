@echo off
REM 烧录分区表脚本
setlocal

REM ESP-IDF 环境配置
set IDF_PYTHON=D:\Espressif\tools\python_env\idf5.5_py3.11_env\Scripts\python.exe
set IDF_PATH=D:\Espressif\v5.5.1\esp-idf
set IDF_TOOLS=%IDF_PYTHON% %IDF_PATH%\tools\idf.py

REM 串口配置（根据实际情况修改）
set COM_PORT=COM63

echo ========================================
echo 烧录分区表到 ESP32-S3
echo ========================================
echo.
echo 使用串口: %COM_PORT%
echo.

%IDF_TOOLS% -p %COM_PORT% partition-table-flash

if errorlevel 1 (
    echo.
    echo 烧录失败！请检查：
    echo 1. ESP32-S3 是否连接到 %COM_PORT%
    echo 2. 串口驱动是否正常
    echo 3. 是否有其他程序占用串口
    goto :end
)

echo.
echo ========================================
echo 烧录成功！
echo ========================================
echo.
echo 请重启ESP32-S3并检查串口输出，应该看到：
echo   "Storage partition found: addr=0x... size=262144 bytes"
echo.

:end
endlocal
pause
