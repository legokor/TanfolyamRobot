@echo off
setlocal

set BUILD_TYPE=%1
if "%BUILD_TYPE%"=="" set BUILD_TYPE=Release

if /I not "%BUILD_TYPE%"=="Debug" if /I not "%BUILD_TYPE%"=="Release" (
    echo Error: Build type must be 'Debug' or 'Release'.
    exit /b 1
)

echo === Configuring CMake for %BUILD_TYPE% ===
cmake -S . --preset %BUILD_TYPE%

echo === Building project (%BUILD_TYPE%) ===
cmake --build --preset %BUILD_TYPE%

endlocal