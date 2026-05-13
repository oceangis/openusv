@echo off
REM ============================================================
REM ArduPilot Rover ESP32-S3 Build & Flash Script
REM ============================================================
REM Usage:
REM   build.bat build              - build only
REM   build.bat flash              - build + flash (COM10)
REM   build.bat flash COM5         - build + flash to specified port
REM   build.bat monitor            - build + flash + monitor
REM   build.bat clean              - full clean build
REM   build.bat erase              - erase flash (fix boot loop)
REM   build.bat menuconfig         - open menuconfig
REM   build.bat all                - clean + build + erase + flash + monitor
REM ============================================================
REM Key fixes applied:
REM   1. MSYSTEM must be cleared (Git Bash/MSYS2 sets it, blocks export.bat)
REM   2. System Python is 3.13 but IDF venv needs 3.11 - prepend IDF Python
REM   3. Must call export.bat to set full toolchain PATH
REM ============================================================

setlocal

REM --- Fix: Git Bash/MSYS2 sets MSYSTEM which blocks export.bat ---
set MSYSTEM=

REM --- ESP-IDF paths ---
set IDF_PATH=D:\Espressif\v5.5.1\esp-idf
set IDF_TOOLS_PATH=D:\Espressif\tools

REM --- Fix: System Python is 3.13 but IDF venv was built with 3.11 ---
set PATH=D:\Espressif\tools\tools\idf-python\3.11.2;%PATH%

REM --- Load ESP-IDF full environment (toolchain, cmake, ninja, etc.) ---
call %IDF_PATH%\export.bat
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] ESP-IDF environment setup failed!
    exit /b 1
)

REM --- Project directory ---
cd /d F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf

REM --- Default COM port (ESP32-S3 USB) ---
set COM_PORT=COM10

echo ========================================
echo ArduPilot Rover ESP32-S3 Build Script
echo IDF: %IDF_PATH%
echo Port: %COM_PORT%
echo ========================================
echo.

if "%1"=="" goto :show_help
if "%1"=="help" goto :show_help
if "%1"=="-h" goto :show_help
if "%1"=="build" goto :build
if "%1"=="flash" goto :flash
if "%1"=="monitor" goto :monitor
if "%1"=="clean" goto :clean
if "%1"=="erase" goto :erase
if "%1"=="menuconfig" goto :menuconfig
if "%1"=="all" goto :all
goto :show_help

:show_help
echo Usage:
echo   build.bat build              - build only
echo   build.bat flash [COMx]       - build + flash
echo   build.bat monitor [COMx]     - build + flash + monitor
echo   build.bat clean              - full clean + rebuild
echo   build.bat erase [COMx]       - erase flash (fix boot loop)
echo   build.bat menuconfig         - open menuconfig
echo   build.bat all [COMx]         - clean + build + erase + flash + monitor
goto :end

:build
echo [BUILD] Starting...
idf.py build
if errorlevel 1 (
    echo [ERROR] Build failed!
    goto :end
)
echo [BUILD] Success! Binary: build\ardupilot_rover_esp32s3.bin
goto :end

:flash
if not "%2"=="" set COM_PORT=%2
echo [BUILD+FLASH] Port: %COM_PORT%
idf.py build
if errorlevel 1 (
    echo [ERROR] Build failed!
    goto :end
)
idf.py -p %COM_PORT% flash
if errorlevel 1 (
    echo [ERROR] Flash failed! Check connection.
    goto :end
)
echo [FLASH] Done!
goto :end

:monitor
if not "%2"=="" set COM_PORT=%2
echo [BUILD+FLASH+MONITOR] Port: %COM_PORT%
idf.py build
if errorlevel 1 (
    echo [ERROR] Build failed!
    goto :end
)
idf.py -p %COM_PORT% flash monitor
goto :end

:clean
echo [CLEAN] Full clean + rebuild...
idf.py fullclean
idf.py build
goto :end

:erase
if not "%2"=="" set COM_PORT=%2
echo [ERASE] Erasing flash on %COM_PORT%...
idf.py -p %COM_PORT% erase-flash
goto :end

:menuconfig
idf.py menuconfig
goto :end

:all
if not "%2"=="" set COM_PORT=%2
echo [ALL] Full pipeline on %COM_PORT%...
echo.
echo [1/5] Clean...
idf.py fullclean
if errorlevel 1 goto :end
echo [2/5] Build...
idf.py build
if errorlevel 1 goto :end
echo [3/5] Erase flash...
idf.py -p %COM_PORT% erase-flash
if errorlevel 1 goto :end
echo [4/5] Flash...
idf.py -p %COM_PORT% flash
if errorlevel 1 goto :end
echo [5/5] Monitor... (Ctrl+] to exit)
idf.py -p %COM_PORT% monitor
goto :end

:end
echo.
echo ========================================
endlocal
