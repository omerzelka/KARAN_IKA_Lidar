@echo off
rem ==========================================================================
rem viewer.bat — WINDOWS PC: STM32 hattindan canli harita + veri dogrulama
rem
rem Kullanim:  viewer.bat            (varsayilan COM6)
rem            viewer.bat COM7       (farkli port)
rem            viewer.bat COM6 --freeray=0
rem
rem Gerekli:   python + pip install pyserial numpy matplotlib
rem Portu bulmak icin: Aygit Yoneticisi -> Baglanti Noktalari -> STLink VCP
rem ==========================================================================
setlocal
set PORT=%1
if "%PORT%"=="" set PORT=COM6

python "%~dp0src\ydlidar_tmini_driver\scripts\scan_map_viewer.py" %PORT% %2 %3
if errorlevel 1 (
    echo.
    echo Hata olustu. Bagimliliklar eksikse:  pip install pyserial numpy matplotlib
    pause
)
endlocal
