@echo off
setlocal EnableExtensions
cd /d "%~dp0.."

powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts\run_case_windows.ps1 ^
  -Config configs\decor_shelf\article.conf ^
  -Results results\decor_shelf ^
  -Verifier scripts\verify_decor_shelf.py
exit /b %ERRORLEVEL%
