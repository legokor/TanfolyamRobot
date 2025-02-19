# TanfolyamRobot - Firmware

## STM32CubeIDE

TODO

## Some code info

### 16 bit timer overflow

All timers on the STM32 are 16 bit, so overflows occur on regular basis. In the code at first glance it might seem that theese events are not handled properly, because we just subtract the start counter value from the end counter value (for time measurement). But this is not the case, because 16 bit arithmetics give the correct result (for example 30000 - 25000 = 60536 which is correct).

## Firmware update

STM32 microcontrollers have a built-in Device Firmware Updater, that makes it possible to upload the firmware over some peripherals. The STM32F103 series only supports this on UART1. The USB connector of the board is connected to an FTDI chip, which connects to the UART1 port of the microcontroller.

Information about the DFU can be found in [AN2606](https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf). The UART DFU protocol is described in [AN3155](https://www.st.com/resource/en/application_note/cd00264342-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf).

There are two ways to run the DFU:
 - Set pins `BOOT0=1` and `BOOT1=0` on reset
 - Jump to the System Memory from the application. It's explained in [this video](https://www.youtube.com/watch?v=cvKC-4tCRgw).

Once the STM32 is in DFU mode, we can use the `stm32flash` utility to update the firmware.
The Windows version is included in the `tools` directory. The Linux version can be downloaded [here](https://sourceforge.net/projects/stm32flash/), but it's also avaiable in Ubuntu's Universe repository.

## Useful documents
 - [STM32F103 Datasheet](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)
 - [[RM008] STM32F103 Reference manual](https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
 - [[AN4776] General-purpose timer cookbook for STM32 microcontrollers](https://www.st.com/resource/en/application_note/dm00236305-generalpurpose-timer-cookbook-for-stm32-microcontrollers-stmicroelectronics.pdf)
 - [[PM0075] STM32F103 Programming manual](https://www.st.com/resource/en/programming_manual/cd00283419-stm32f10xxx-flash-memory-microcontrollers-stmicroelectronics.pdf)
 - [[AN2606] STM32 microcontroller system memory boot mode](https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf)
 - [[AN3155] USART protocol used in the STM32 bootloader](https://www.st.com/resource/en/application_note/cd00264342-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf)
 - [TCS3200 color sensor datasheet](https://www.mouser.com/catalog/specsheets/tcs3200-e11.pdf)
 - [Ultrasonic sensor "datasheet"](http://www.energiazero.org/arduino_sensori/Arduino%20ultrasonic%20sensor%20(HC-SR04%20or%20HY-SRF05).pdf)
 - [SG90 servo datasheet](http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf)
 - [DRV8833 Dual H-Bridge Motor Driver datasheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf?ts=1604564346135&ref_url=https%253A%252F%252Fwww.ti.com%252Fmotor-drivers%252Fbrushed-dc-bdc-drivers%252Fproducts.html)
