@echo off
echo ========================================
echo CLEARING PYTHON CACHE
echo ========================================
echo.

cd C:\Users\mort8\ffa-simulator

echo Deleting __pycache__ directories...
for /d /r . %%d in (__pycache__) do @if exist "%%d" rd /s /q "%%d"

echo Deleting .pyc files...
del /s /q *.pyc

echo.
echo ========================================
echo CACHE CLEARED
echo ========================================
echo.
echo Now start FFA Simulator normally
echo.
pause
