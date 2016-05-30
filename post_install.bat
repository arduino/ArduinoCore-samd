@echo off
set ARGS=/A /SE /SW /SA
if "%PROCESSOR_ARCHITECTURE%" == "AMD64" (
  drivers\dpinst-amd64.exe %ARGS%
) ELSE IF "%PROCESSOR_ARCHITEW6432%" == "AMD64" (
  drivers\dpinst-amd64.exe %ARGS%
) ELSE (
  drivers\dpinst-x86.exe %ARGS%
)
exit /b 0
