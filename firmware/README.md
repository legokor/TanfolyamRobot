# TanfolyamRobot - Firmware

## STM32CubeIDE

TODO

## Firmware update

STM32 microcontrollers have a built-in Device Firmware Updater, that makes it possible to upload the firmware over some peripherals. The STM32F103 series only supports this on UART1. The USB connector of the board is connected to an FTDI chip, which connects to the UART1 port of the microcontroller.

Information about the DFU can be found in [AN2606](https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf). The UART DFU protocol is described in [AN3155](https://www.st.com/resource/en/application_note/cd00264342-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf).

There are two ways to run the DFU:
 - Set pins `BOOT0=1` and `BOOT1=0` on reset
 - Jump to the System Memory from the application. It's explained in [this video](https://www.youtube.com/watch?v=cvKC-4tCRgw). This is implemented in `Drivers/dfu/dfu.c`

##### Linux
It's recommended to use the `stm32flash` utility to update the firmware. The `upload.sh` script makes it easier to use. It can be downloaded [here](https://sourceforge.net/projects/stm32flash/), but it's also avaiable in Ubuntu's Universe repository.

##### Windows
TODO: check if `stm32flash` works


## Useful documents
 - [STM32F103 Datasheet](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)
 - [(RM008) STM32F103 Reference manual](https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
 - [(PM0075) STM32F103 Programming manual](https://www.st.com/resource/en/programming_manual/cd00283419-stm32f10xxx-flash-memory-microcontrollers-stmicroelectronics.pdf)
 - [(AN2606) STM32 microcontroller system memory boot mode](https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf)
 - [(AN3155) USART protocol used in the STM32 bootloader](https://www.st.com/resource/en/application_note/cd00264342-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf)