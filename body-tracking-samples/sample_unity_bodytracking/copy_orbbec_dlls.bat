@echo off
REM Copy Orbbec K4A Wrapper DLLs to Unity Plugins directory
REM Run this before opening Unity (Unity must be closed)

set SRC=C:\OrbbecSDK_K4A_Wrapper_v1.10.5_windows_202510212040\bin
set DST=%~dp0Assets\Plugins

echo Orbbec DLL Copy Script for Unity
echo =================================
echo Source: %SRC%
echo Destination: %DST%
echo.

if not exist "%SRC%" (
    echo ERROR: Source directory not found: %SRC%
    pause
    exit /b 1
)

if not exist "%DST%" (
    echo ERROR: Destination directory not found: %DST%
    echo Make sure you are running this from the Unity project root.
    pause
    exit /b 1
)

echo WARNING: Make sure Unity is closed before running this script!
echo.
pause

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
echo Done! You can now open Unity.
pause
