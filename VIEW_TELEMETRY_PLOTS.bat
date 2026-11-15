@echo off
setlocal
cd /d "C:\Users\User\.gemini\antigravity\scratch\ros2_ws"
cls
echo =====================================================================
echo   GENERATING SCIENTIFIC TELEMETRY & SPEED PLOTS (LOGS VIEWER)
echo =====================================================================
echo.
echo Processing logs/delivery_telemetry.csv...
python scripts\plot_telemetry_analytics.py
if exist "logs\delivery_telemetry_report.png" (
    echo Opening generated report image...
    start "" "logs\delivery_telemetry_report.png"
)
pause
