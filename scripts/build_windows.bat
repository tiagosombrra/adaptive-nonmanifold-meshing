@echo off
setlocal EnableExtensions
cd /d "%~dp0.."

where cmake.exe >nul 2>&1
if not "%ERRORLEVEL%"=="0" (
  echo ERROR: cmake.exe was not found in PATH.
  exit /b 1
)

set "CXX_COMPILER="
for /f "delims=" %%G in ('where g++.exe 2^>nul') do (
  if not defined CXX_COMPILER set "CXX_COMPILER=%%G"
)

if not defined CXX_COMPILER (
  echo ERROR: g++.exe was not found in PATH.
  exit /b 1
)

echo Using C++ compiler: %CXX_COMPILER%
echo Removing any previous CMake build directory to avoid compiler-cache reuse.

if exist build rmdir /s /q build

cmake -S . -B build ^
  -G "MinGW Makefiles" ^
  "-DCMAKE_CXX_COMPILER=%CXX_COMPILER%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DAP_MESH_USE_MPI=OFF ^
  -DAP_MESH_USE_OPENMP=OFF ^
  -DAP_MESH_ENABLE_LTO=OFF ^
  -DAP_MESH_ENABLE_WARNINGS=OFF ^
  -DAP_MESH_STATIC_MINGW_RUNTIME=ON

set "CONFIGURE_RC=%ERRORLEVEL%"
if not "%CONFIGURE_RC%"=="0" exit /b 1

cmake --build build --parallel 1 --clean-first
set "BUILD_RC=%ERRORLEVEL%"
if not "%BUILD_RC%"=="0" exit /b 1

exit /b 0
