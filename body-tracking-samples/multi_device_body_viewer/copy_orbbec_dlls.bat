@echo off
REM Copy Orbbec K4A Wrapper DLLs to build output directory
REM Run this after building multi_device_body_viewer

set SRC=C:\OrbbecSDK_K4A_Wrapper_v1.10.5_windows_202510212040\bin
set DST=%~dp0\build\bin\Release

echo Orbbec DLL Copy Script
echo ======================
echo Source: %SRC%
echo Destination: %DST%
echo.

if not exist "%SRC%" (
    echo ERROR: Source directory not found: %SRC%
    pause
    exit /b 1
)

if not exist "%DST%" (
    echo Creating destination directory...
    mkdir "%DST%"
)

echo Copying K4A Wrapper DLLs...
copy /Y "%SRC%\k4a.dll" "%DST%\"
copy /Y "%SRC%\k4arecord.dll" "%DST%\"
copy /Y "%SRC%\depthengine_2_0.dll" "%DST%\"

echo Copying Orbbec SDK dependencies...
copy /Y "%SRC%\OrbbecSDK.dll" "%DST%\"
copy /Y "%SRC%\ob_usb.dll" "%DST%\"
copy /Y "%SRC%\live555.dll" "%DST%\"
copy /Y "%SRC%\OrbbecSDKConfig_v1.0.xml" "%DST%\"

echo.
echo Verifying k4a.dll size (should be ~283KB for Orbbec)...
for %%F in ("%DST%\k4a.dll") do echo k4a.dll: %%~zF bytes

echo.
echo Done!
pause
