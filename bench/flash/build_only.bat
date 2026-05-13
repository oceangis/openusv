@echo off
set MSYSTEM=
set IDF_PATH=D:\Espressif\v5.5.1\esp-idf
set IDF_TOOLS_PATH=D:\Espressif\tools
set PATH=D:\Espressif\tools\tools\idf-python\3.11.2;%PATH%
echo [1] calling export.bat...
call %IDF_PATH%\export.bat
echo [2] export done, errorlevel=%ERRORLEVEL%
cd /d F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf
echo [3] starting build...
idf.py build
echo [4] build done, errorlevel=%ERRORLEVEL%
