@echo off
setlocal enabledelayedexpansion

set FILE=..\Application\application.c



findstr /C:"MANUAL_IP TRUE" "%FILE%" >nul && (
    echo Manual IP address mode detected
	goto :eof
)

findstr /C:"SERVER_IP" "%FILE%" >nul
if %errorlevel% neq 0 (
	echo SERVER_IP line not found in the file
	goto :eof
)



rem Run ipconfig command and filter for WiFi interface IP address
for /f "tokens=2 delims=:" %%a in ('ipconfig ^| findstr /c:"Wireless LAN adapter Wi-Fi" /c:"IPv4 Address"') do (
    set "ip=%%a"
)
rem Remove leading and trailing spaces
set "ip=%ip:~1%"
set "ip=%ip: =%"

echo WiFi Interface IP Address: %ip%
echo Auto setting SERVER_IP to %ip%

set NEW_SERVER_IP=%ip%

setlocal DisableDelayedExpansion
FOR /F "delims=" %%L in ('findstr /N "^" "%FILE%"') DO (
	set "line=%%L"
	setlocal EnableDelayedExpansion
	set "line=!line:*:=!" & rem Remove all characters to the first colon
	echo !line! | findstr /c:"SERVER_IP" >nul
	if !errorlevel! equ 0 (
		echo const char^* SERVER_IP = "%NEW_SERVER_IP%";>> temp.c
		echo !line! | findstr /c:"%NEW_SERVER_IP%" >nul
		if !errorlevel! equ 0 (
			echo SERVER_IP is already up to date
			del temp.c
			goto :eof
		)
	) else (
		echo(!line!>> temp.c
	)
	endlocal
)
setlocal enabledelayedexpansion
move /Y temp.c %FILE% >nul
echo Server IP address set to %NEW_SERVER_IP%.
