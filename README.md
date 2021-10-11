# TanfolyamRobot

## Fejlesztőkörnyezet

A firmware fejlesztéséhez az STM32CubeIDE fejlesztőkörnyezetet használjuk, mely regisztráció után letölthető az [STMicroelectronics weboldaláról](https://www.st.com/en/development-tools/stm32cubeide.html).

Tesztelt operációs rendszerek: 
 * Windows 10
 * Ubuntu 18.04
 * Ubuntu 20.04
 * Pop!_OS 21.04

### További lépések Linux esetén
A firmware letöltése a robotra virtuális soros porton keresztül történik. A soros portok csak akkor használhatóak, ha a felhasználó tagja `dialout` csoportnak. 

```
sudo adduser $USER dialout
```

Ezután oprendszertől függően ki és be kell jelentkezni, vagy újra kell indítani a gépet.

## Projekt importálása
GitHubon a [Releases](https://github.com/legokor/TanfolyamRobot/releases) alatt töltsük le a projekt aktuális verzióját.


