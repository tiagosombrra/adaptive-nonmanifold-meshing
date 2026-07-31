@echo off
setlocal EnableExtensions
cd /d "%~dp0.."

powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts\run_case_windows.ps1 ^
  -Config configs\book\article.conf ^
  -Results results\book ^
  -Verifier scripts\verify_book.py
exit /b %ERRORLEVEL%
