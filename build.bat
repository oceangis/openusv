@echo off
REM ArduPilot Rover ESP32-S3 编译脚本
REM 使用方法: build.bat [clean|build|flash|monitor|all]

setlocal

REM ESP-IDF 环境配置
set IDF_PYTHON=D:\Espressif\tools\python_env\idf5.5_py3.11_env\Scripts\python.exe
set IDF_PATH=D:\Espressif\v5.5.1\esp-idf
set IDF_TOOLS=%IDF_PYTHON% %IDF_PATH%\tools\idf.py

REM 默认串口（根据实际情况修改）
set COM_PORT=COM3

echo ========================================
echo ArduPilot Rover ESP32-S3 Build Script
echo ========================================
echo.

if "%1"=="" goto :show_help
if "%1"=="help" goto :show_help
if "%1"=="-h" goto :show_help
if "%1"=="clean" goto :clean
if "%1"=="build" goto :build
if "%1"=="flash" goto :flash
if "%1"=="erase" goto :erase
if "%1"=="monitor" goto :monitor
if "%1"=="all" goto :all
goto :show_help

:show_help
echo 使用方法:
echo   build.bat clean    - 完全清理构建目录
echo   build.bat build    - 编译项目
echo   build.bat erase    - 擦除Flash (解决启动循环问题)
echo   build.bat flash    - 烧录固件到开发板
echo   build.bat monitor  - 打开串口监视器
echo   build.bat all      - 清理+编译+擦除+烧录+监控 (完整流程)
echo.
echo 注意: 请根据实际情况修改脚本中的 COM_PORT 变量
goto :end

:clean
echo [1/1] 清理构建目录...
%IDF_TOOLS% fullclean
if errorlevel 1 (
    echo 清理失败！
    goto :end
)
echo 清理完成！
goto :end

:build
echo [1/1] 开始编译...
%IDF_TOOLS% build
if errorlevel 1 (
    echo 编译失败！请检查错误信息。
    goto :end
)
echo.
echo 编译成功！
echo 二进制文件位置: build\ardupilot_rover_esp32s3.bin
goto :end

:erase
echo [1/1] 擦除Flash...
echo 使用串口: %COM_PORT%
%IDF_TOOLS% -p %COM_PORT% erase-flash
if errorlevel 1 (
    echo 擦除失败！请检查串口连接和权限。
    goto :end
)
echo Flash擦除完成！
goto :end

:flash
echo [1/1] 烧录固件...
echo 使用串口: %COM_PORT%
%IDF_TOOLS% -p %COM_PORT% flash
if errorlevel 1 (
    echo 烧录失败！请检查串口连接和权限。
    goto :end
)
echo 烧录完成！
goto :end

:monitor
echo [1/1] 打开串口监视器...
echo 使用串口: %COM_PORT%
echo 按 Ctrl+] 退出监视器
%IDF_TOOLS% -p %COM_PORT% monitor
goto :end

:all
echo ========================================
echo 执行完整编译流程
echo ========================================
echo.

echo [1/5] 清理构建目录...
%IDF_TOOLS% fullclean
if errorlevel 1 (
    echo 清理失败！
    goto :end
)
echo.

echo [2/5] 编译项目...
%IDF_TOOLS% build
if errorlevel 1 (
    echo 编译失败！终止流程。
    goto :end
)
echo.

echo [3/5] 擦除Flash...
echo 使用串口: %COM_PORT%
%IDF_TOOLS% -p %COM_PORT% erase-flash
if errorlevel 1 (
    echo 擦除失败！终止流程。
    goto :end
)
echo.

echo [4/5] 烧录固件...
%IDF_TOOLS% -p %COM_PORT% flash
if errorlevel 1 (
    echo 烧录失败！终止流程。
    goto :end
)
echo.

echo [5/5] 打开串口监视器...
echo 按 Ctrl+] 退出监视器
%IDF_TOOLS% -p %COM_PORT% monitor
goto :end

:end
echo.
echo ========================================
endlocal
