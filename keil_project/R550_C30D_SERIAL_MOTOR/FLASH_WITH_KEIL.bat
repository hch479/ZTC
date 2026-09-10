@echo off
setlocal
set "KEIL_UV4=C:\Users\User\Desktop\keil5\UV4\UV4.exe"
set "PROJECT=%~dp0USER\WHEELTEC.uvprojx"
set "FLASH_LOG=%~dp0OBJ_SERIAL\keil_flash.log"

if not exist "%KEIL_UV4%" exit /b 3
if not exist "%~dp0OBJ_SERIAL\C30D_SERIAL_MOTOR.axf" exit /b 4

"%KEIL_UV4%" -f "%PROJECT%" -t"C30D_SERIAL_MOTOR" -o"%FLASH_LOG%" -j0
set "FLASH_RESULT=%ERRORLEVEL%"
if exist "%FLASH_LOG%" type "%FLASH_LOG%"
echo Keil flash exit code: %FLASH_RESULT%
exit /b %FLASH_RESULT%
