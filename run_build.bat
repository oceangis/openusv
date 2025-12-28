@echo off
set MSYSTEM=
set MSYSCON=
set IDF_PYTHON_ENV_PATH=D:\Espressif\tools\python_env\idf5.5_py3.11_env
set IDF_PATH=D:\Espressif\v5.5.1\esp-idf
cd /d f:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf
D:\Espressif\tools\python_env\idf5.5_py3.11_env\Scripts\python.exe D:\Espressif\v5.5.1\esp-idf\tools\idf.py build
