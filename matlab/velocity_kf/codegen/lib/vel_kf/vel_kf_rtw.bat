@echo off

@if not "%MINGW_ROOT%" == "" (@set "PATH=%PATH%;%MINGW_ROOT%")

cd .

if "%1"=="" ("C:\PROGRA~1\MATLAB\R2020b\bin\win64\gmake"  -f vel_kf_rtw.mk all) else ("C:\PROGRA~1\MATLAB\R2020b\bin\win64\gmake"  -f vel_kf_rtw.mk %1)
@if errorlevel 1 goto error_exit

exit 0

:error_exit
echo The make command returned an error of %errorlevel%
exit 1