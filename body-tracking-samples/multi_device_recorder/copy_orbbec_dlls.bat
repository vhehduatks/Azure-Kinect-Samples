@echo off
REM Copy Orbbec K4A Wrapper DLLs to build output directory
REM Run this after building to replace Azure Kinect DLLs with Orbbec versions

set ORBBEC_SDK=C:\OrbbecSDK_K4A_Wrapper_v1.10.5_windows_202510212040\bin
set BUILD_DIR=%~dp0build\bin\Release

echo.
echo === Orbbec DLL Copy Script for multi_device_recorder ===
echo.

if not exist "%ORBBEC_SDK%" (
    echo ERROR: Orbbec SDK not found at %ORBBEC_SDK%
    echo Please update the ORBBEC_SDK path in this script.
    exit /b 1
)

if not exist "%BUILD_DIR%" (
    echo Build directory not found: %BUILD_DIR%
    echo Creating directory...
    mkdir "%BUILD_DIR%"
)

echo Copying Orbbec K4A Wrapper DLLs...
echo From: %ORBBEC_SDK%
echo To:   %BUILD_DIR%
echo.

REM Core K4A replacement DLLs
copy /Y "%ORBBEC_SDK%\k4a.dll" "%BUILD_DIR%\"
copy /Y "%ORBBEC_SDK%\k4arecord.dll" "%BUILD_DIR%\"
copy /Y "%ORBBEC_SDK%\depthengine_2_0.dll" "%BUILD_DIR%\"

REM Orbbec SDK dependencies
copy /Y "%ORBBEC_SDK%\OrbbecSDK.dll" "%BUILD_DIR%\"
copy /Y "%ORBBEC_SDK%\ob_usb.dll" "%BUILD_DIR%\"
copy /Y "%ORBBEC_SDK%\live555.dll" "%BUILD_DIR%\"

REM Orbbec config
copy /Y "%ORBBEC_SDK%\OrbbecSDKConfig_v1.0.xml" "%BUILD_DIR%\"

echo.
echo Done! Verifying k4a.dll size...
for %%A in ("%BUILD_DIR%\k4a.dll") do echo k4a.dll size: %%~zA bytes (should be ~283KB for Orbbec)
echo.
echo If k4a.dll is ~651KB, it's still the Azure Kinect version.
echo.
pause
