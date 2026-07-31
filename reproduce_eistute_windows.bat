@echo off
setlocal
cd /d "%~dp0"

echo ============================================================
echo GRSI representative reproduction
echo Paper Figure 7 - Eistute model
echo ============================================================
echo.

call scripts\build_windows.bat
if errorlevel 1 exit /b 1

call scripts\run_eistute_windows.bat
if errorlevel 1 exit /b 1

echo.
echo Representative Eistute result reproduced successfully.
echo Generated data: results\eistute
exit /b 0
