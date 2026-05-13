@echo off
REM Run MAVProxy as a pure bridge: COM58 <-> UDP 14550 + tlog recording.
REM No interactive commands. Daemon mode keeps it running until killed.
set PYTHONIOENCODING=utf-8
cd /d F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf

D:\Espressif\tools\python_env\idf5.5_py3.11_env\Scripts\python.exe ^
  D:\Espressif\tools\python_env\idf5.5_py3.11_env\Scripts\mavproxy.py ^
  --master=COM58 ^
  --baudrate=115200 ^
  --out=udpout:127.0.0.1:14550 ^
  --logfile=session.tlog ^
  --source-system=254 ^
  --source-component=0 ^
  --daemon
