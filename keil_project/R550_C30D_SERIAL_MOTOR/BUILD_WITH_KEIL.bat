@echo off
setlocal
set "KEIL_UV4=C:\Users\User\Desktop\keil5\UV4\UV4.exe"
set "PROJECT=%~dp0USER\WHEELTEC.uvprojx"
set "BUILD_LOG=%~dp0OBJ_SERIAL\keil_build.log"

if not exist "%KEIL_UV4%" (
    echo [ERROR] Keil not found: %KEIL_UV4%
    exit /b 3
)

if not exist "%~dp0OBJ_SERIAL" mkdir "%~dp0OBJ_SERIAL"
"%KEIL_UV4%" -b "%PROJECT%" -t"C30D_SERIAL_MOTOR" -o"%BUILD_LOG%"
set "BUILD_RESULT=%ERRORLEVEL%"

if exist "%BUILD_LOG%" type "%BUILD_LOG%"
echo.
echo Keil exit code: %BUILD_RESULT%
exit /b %BUILD_RESULT%
