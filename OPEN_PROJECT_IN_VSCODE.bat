@echo off
TITLE Opening Project in Visual Studio Code
echo =====================================================================
echo   Opening ROS 2 Mapless Café Delivery Bot Workspace in VS Code...
echo =====================================================================
cd /d "%~dp0"
call code .
if %ERRORLEVEL% EQU 0 (
    echo Workspace opened successfully in VS Code!
) else (
    echo [NOTE] VS Code CLI 'code' command not found in default PATH.
    echo Trying manual path...
    if exist "D:\Microsoft VS Code\bin\code.cmd" (
        call "D:\Microsoft VS Code\bin\code.cmd" .
    ) else (
        echo Please open VS Code and use File -> Open Folder -> %~dp0
    )
)
timeout /t 3 >nul
