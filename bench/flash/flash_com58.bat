@echo off
set MSYSTEM=
set IDF_PATH=D:\Espressif\v5.5.1\esp-idf
set IDF_TOOLS_PATH=D:\Espressif\tools
set PATH=D:\Espressif\tools\tools\idf-python\3.11.2;%PATH%
call %IDF_PATH%\export.bat >nul 2>&1
cd /d F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf
echo === BUILD START ===
idf.py build
echo === BUILD DONE errorlevel=%ERRORLEVEL% ===
if errorlevel 1 exit /b 1
echo === FLASH START COM58 ===
idf.py -p COM58 flash
echo === FLASH DONE errorlevel=%ERRORLEVEL% ===
