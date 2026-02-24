# LEGO Kör - TanfolyamRobot

## Fejlesztő környezet

Korábban az STMicroelectronics által biztosított `STM32CubeIDE`-t használtuk a firmware fejlesztéséhez, azonban ez egy elég nehézkes és lassú eszköz. Ezért a tanfolyam során áttérünk egy könnyebben használható és gyorsabb parancssoros toolchain-re, az `STM32CubeCLT`-re. Ezt a toolchain-t a Visual Studio Code nevű (továbbiakban `VSCode`) kód szerkesztővel fogjuk használni, így a teljes fejlesztés (fordítás, feltöltés) nagyon könnyen indítható lesz néhány előre beállított `task`-kal.

## Toolchain

Az előző fejezetben említett módon a firmware fejlesztéséhez az STMicroelectronics parancsoros eszközeit használjuk. Sajnos ezek letöltése regisztrációhoz kötött, ezért feltöltöttük a telepítő fájlokat [a körös drive-ra](https://drive.google.com/drive/folders/1nJ2nfvr9kbIkVAWmcgEXpbxK1OUlX8sQ?usp=sharing), így regisztráció nélkül is telepíteni tudjátok - persze teljesen megértjük, ha nem akartok random drive-os fájlokat telepíteni, viszont ez esetben muszáj regisztrálnotok az [STMicroelectronics weboldalán](https://www.st.com/en/development-tools/stm32cubeclt.html), és onnan letölteni.

Letöltés után a telepítő fájlt futtatva telepítsétek a `STM32CubeCLT`-t egy kényelmes helyre, például `C:\STM32CubeCLT`-be.

**FONTOS:** az `STM32CubeCLT` telepítése után újra kell indítani a `VSCode`-ot amennyiben az már meg volt nyitva, különben a `VSCode` nem fogja látni a telepített toolchain-t. Ezen felül Linux rendszerek esetében előfordulhat, hogy a gépet is újra kell indítani, hogy a megfelelő környezeti változók beálljanak.

Jelenleg Windows 10/11-en és Ubuntu-n teszteljük a szoftvert. Más (Linux alapú) platformokon is valószínűleg működni fog - de ezt nem tudjuk garantálni.

## Robot illesztőszoftver

A robotra a firmware letöltése virtuális soros porton történik, a roboton az USB csatlakozó egy `FT232RL` chiphez kapcsolódik.

Előfordulhat Windows esetében, hogy az nem telepíti automatikusan a virtuális soros port driverét. Erre az esetre a fentebbi drive linken elérhető Windows zip-ben találtok egy plusz driver telepítőt, de amúgy ezt is le tudjátok tölteni az [FTDI weboldaláról](https://ftdichip.com/wp-content/uploads/2021/08/CDM212364_Setup.zip), majd pedig kibontani és telepíteni.

## Kiinduló projekt letöltése, megnyitása

A kiinduló projektet a [GitHub repojából](https://github.com/legokor/TanfolyamRobot) tudjuk letölteni, ehhez az oldal megnyitása után jobb oldalt a `Releases` fül alatt kell kiválasztanunk a **legújabb kiadást**, majd a `tanfrobot.zip` fájlra kell kattintani.

A letöltött fájlt helyezzük el egy kényelmes helyre, például `Documents/Lego_Tanfolyam` mappába, majd bontsuk ki. A kibontás után nyissuk meg a `VSCode`-ot, és a `File -> Open Folder...` menüponttal nyissuk meg a kibontott `firmware` mappát (mely többek közt az `Application`, `Drivers`, `Tools` könyvtárakat tartalmazza).

*Extra tipp:* a `VSCode`-ban érdemes telepíteni az `ms-vscode.cpptools` nevű bővítményt, mely C/C++ nyelv támogatást ad a szerkesztőhöz, így például szintaxis kiemelést és automatikus kiegészítést biztosít.

--------------------------------------------

--------------------------------------------

## *Csak a tanfolyamon kell:* projekt áttekintése, build és upload

Az imént megnyitott projektünk fájljait az IDE bal oldalán láthatjuk. A kód számunkra lényegi része a `tanfolyamrobot/Application/src/app.c` fájl lesz. Itt kell majd megoldanunk a tanfolyam során a feladatokat. Az itt található `app_main()` függvény gombnyomásra indul. Ha ebből a függvényből visszatér a programunk, akkor a robot leáll és a kijelzőn megjeleníti a visszatérési értéket.

A kód fordításához és a robotra való feltöltéséhez a `VSCode`-ban előre beállítottunk néhány `task`-ot, melyeket a `CTRL+SHIFT+B` billentyűkombinációval tudtok elindítani. E billentyűkombináció megnyomása után egy legördülő menüből választhatjátok ki, hogy mit szeretnétek csinálni - számunkra a `CMake Build` és az `STM32 Flash Firmware (FTDI)` lesz a fontos. Előbbi lefordítja a kódot, utóbbi pedig feltölti a robotra. A kiválasztás után egy újabb kérdés fog megjelenni, hogy melyik konfigurációt szeretnénk használni - válasszuk a `Release`-t, majd nyomjunk egy entert.

**FONTOS:** a `CMake Build`-et mindig futtassátok le a `STM32 Flash Firmware (FTDI)` előtt, különben a régi kód fog futni a roboton, nem pedig a legutóbb módosított verzió.

