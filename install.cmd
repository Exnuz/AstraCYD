@echo off
setlocal
if "%~1"=="" (
    echo Usage: install.cmd --probe [--port COMx]
    echo        install.cmd --backup-only [--port COMx]
    echo        install.cmd --install [--port COMx]
    exit /b 2
)
if not exist "%~dp0.venv\Scripts\python.exe" (
    if defined ASTRACYD_PYTHON (
        "%ASTRACYD_PYTHON%" -m venv "%~dp0.venv"
    ) else (
        where py >nul 2>&1
        if errorlevel 1 (
            python -m venv "%~dp0.venv"
        ) else (
            py -3 -m venv "%~dp0.venv"
        )
    )
    if errorlevel 1 exit /b 1
)
"%~dp0.venv\Scripts\python.exe" -c "import esptool; assert esptool.__version__ == '4.9.0'" >nul 2>&1
if errorlevel 1 (
    "%~dp0.venv\Scripts\python.exe" -m pip install esptool==4.9.0
    if errorlevel 1 exit /b 1
)
"%~dp0.venv\Scripts\python.exe" "%~dp0install.py" %*
exit /b %errorlevel%
