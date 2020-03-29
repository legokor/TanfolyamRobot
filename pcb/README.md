# TanfolyamRobot - PCB design

_Note: schematic net and pin names are formatted like_ `this`.

## Microcontroller

The main MCU is an STM32F103C8T6 in TQFP64 package. There's an option to add an ESP8266 module which communicates with the main MCU via UART.

**The full STM32 pinout and pin assignments can be found in [STM32F103_pin_assignment.xls](STM32F103_pin_assignment.xls)**

### Programming

The STM32 can be programmed using the Serial Wire Debug interface or the integrated Device Firmware Updater. The F103's DFU only supports USART1, so that's connected to an FTDI FT232RL USB-UART converter.

In order to use the DFU, the MCU must be booted in System memory mode by pulling the `BOOT0` pin high at reset.
`BOOT0` is controlled by:
 * the FTDI chip's `RSTN` reset signal, which is high when USB is connected.
 * an FTDI output pin controlled by PC software. The pin can be selected by solder jumpers.


### Reset

Both controllers are connected to the same reset signal, which is a combination of the following reset sources:
 * The reset button
 * Debugger port
 * USB 5V connection
 * an FTDI output pin controlled by PC software. The pin can be selected by solder jumpers.

## Power supply

The battery input connects through a polarity protection MOSFET to the `VCC` rail, which drives the motors and the 5V regulator. Therefore the battery voltage must be at least 5V + the regulator's dropout voltage, and can be as high as the motors' and the motor driver's maximum operating voltage. The polarity protection MOSFET's absolute maximum V<sub>GS</sub> and V<sub>DS</sub> must be higher than the supply voltage.

The `+5V` rail powers the FTDI chip, LCD, ultrasonic sensor, motor encoders, and the servo motor. Therefore the ultrasonic sensor and motor encoder outputs shall be connected to a 5V tolerant MCU pin or level-shifted to 3.3V.

The `+3V3` rail powers the rest of the system: STM32, ESP, color sensor, and it's also the `VCCIO` supply of the FTDI chip.


## Motors

The two motors are driven by a DRV8833 module. For maximum flexibility all of its input ports shall be connected to PWM cabable MCU pins, preferably using the same timer. The driver indicates overcurrent/overheat faults on the `FAULTN` pin, and the whole module can be enabled/disabled by the `SLEEPN` pin.

The STM32's timers have a built-in encoder pulse counter module, which uses the CH1-CH2 channel pair. The motors' encoder pins therefore shall be connected to these timer channels.

The servo output has a 3 pin header with 5V supply and a PWM pin.


## Sensors, LCD

The TCS3200 color sensor has a frequency output. 2 config pins activate the RGB color filters, and 2 config pins set the output frequency scaling. The output pin should be connected to a timer of the MCU. The timer's frequency is not critical as the output full scale frequency can be configured between 12 and 600 kHz.

The generic ultrasonic sensor has a trigger and an echo pin. `ECHO` shall be connected to an interrupt-capable MCU pin.

The character LCD screen operates in 4 bit mode. In this configuration only the `D4`-`D7`, `EN` and `RST` pins are used. Contrast can be set by a trimmer potentiometer on the board. The backlight is also controlled by the MCU, but the control signal is pulled up, so it's on by default.

# MCU connection requirement summary

_I know this looks terrible, but it's my only workaround for GitHub's alternating row colors :disappointed:_

| Component     | Pin/Schematic net      | Type             | MCU requiurement                              |
|---------------|------------------------|------------------|-----------------------------------------------|
| DRV8833       | `IN1`, `IN2`           | PWM/fix input    | 1 PWM, 1 PWM/fix, same timer for all channels |
|||||
|               | `IN3`, `IN4`           | PWM/fix input    | 1 PWM, 1 PWM/fix, same timer for all channels |
|||||
|               | `SLEEPN`               | input            | -                                             |
|||||
|               | `FAULTN`               | output, OC       | -                                             |
|||||
| Encoder       | `ENC1_A`, `ENC1_B`     | output           | Timer CH1 - CH2                               |
|||||
|               | `ENC2_A`, `ENC2_B`     | output           | Timer CH1 - CH2                               |
|||||
| Servo         | `SERVO`                | PWM input        | Timer                                         |
|||||
| TCS3200       | `S1`-`S4`              | input            | -                                             |
|||||
|               | `OUT`                  | frequency output | Timer                                         |
|||||
| Generic US    | `TRIG`                 | input            | -                                             |
|||||
|               | `ECHO`                 | output           | Interrupt                                     |
|||||
| LCD           | `RST`,`EN`, `D4`-`D7`  | input            | -                                             |
|||||
|               | `BACKLIGHT`            | input            | Preferably PWM                                |
|||||
| ESP8266       | `RX`, `TX`             | UART             | Any UART                                      |
|||||
| SWD connector | `SWCLK`, `SWDIO`, `SWO`| Debug            | Corresponding debugger pins                   |
|||||
| FT232RL       | `RX`, `TX`             | UART             | UART1                                         |






