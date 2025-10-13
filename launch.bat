@echo off

REM Get the directory of this script
set "SCRIPT_DIR=%~dp0"
set "PYTHONPATH=%SCRIPT_DIR%"

REM Define the virtual environment path
set "PATH=%SCRIPT_DIR%.venv\Scripts"
if not exist "%PATH%" (
    echo ERROR: Virtual environment not found at "%PATH%"
    exit /b 1
)

REM Determine which world file to launch
if "%~1"=="" (
    set "WORLD_FILE=robots\e_puck\worlds\my_world.wbt"
) else (
    set "WORLD_FILE=robots\e_puck\worlds\%~1"
)

REM Check if the world file exists
echo Checking world file: "%WORLD_FILE%"
if not exist "%WORLD_FILE%" (
    echo ERROR: World file not found ...
    exit /b 1
)

REM Launch Webots
"C:\Users\nobil\AppData\Local\Programs\Webots\msys64\mingw64\bin\webotsw.exe" "%SCRIPT_DIR%%WORLD_FILE%"