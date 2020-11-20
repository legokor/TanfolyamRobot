@Echo Off

:: Baud rate for communication with the firmware
set /A APPLICATION_BAUD=115200

:: Serial command for entering DFU mode
set DFU_COMMAND=ENTER_DFU

:: Baud rate for stm32flash utility
set /A UPLOAD_BAUD=500000

:: Count the number of arguments
set /A argC=0
for %%x in (%*) do Set /A argC+=1

if NOT %argC% == 1 (
	echo usage: %0 port
	exit /b
)

::  Set baud rate and send ENTER_DFU command
mode %1 BAUD=%APPLICATION_BAUD% PARITY=n DATA=8
echo Enter DFU mode...
echo %DFU_COMMAND% > %1

:: Wait for system reset
:: Sleep about 1-2 s. It's ridiculous, but this is the best way to do it on Windows..
ping -n 3 127.0.0.1 >nul

:: Measure upload time
set start=%time%

:: Flash MCU
stm32flash -b %UPLOAD_BAUD% -w ../Debug/tanfolyamrobot.bin -v -g 0x0 %1

:: Calculate and print upload time. Yeah.. I know..
set end=%time%
set options="tokens=1-4 delims=:.,"
for /f %options% %%a in ("%start%") do set start_h=%%a&set /a start_m=100%%b %% 100&set /a start_s=100%%c %% 100&set /a start_ms=100%%d %% 100
for /f %options% %%a in ("%end%") do set end_h=%%a&set /a end_m=100%%b %% 100&set /a end_s=100%%c %% 100&set /a end_ms=100%%d %% 100

set /a hours=%end_h%-%start_h%
set /a mins=%end_m%-%start_m%
set /a secs=%end_s%-%start_s%
set /a ms=%end_ms%-%start_ms%

if %ms% lss 0 set /a secs = %secs% - 1 & set /a ms = 100%ms%
if %secs% lss 0 set /a mins = %mins% - 1 & set /a secs = 60%secs%
if %mins% lss 0 set /a hours = %hours% - 1 & set /a mins = 60%mins%
if %hours% lss 0 set /a hours = 24%hours%
if 1%ms% lss 100 set ms=0%ms%
set /a totalsecs = %hours%*3600 + %mins%*60 + %secs%

echo Upload time: %totalsecs%.%ms% s
