# UART communication description between the STM32 and the ESP8266
The communication happens using bidirectional asynchronous UART at 115200 bits/s.

## ESP8266 -> STM32
The communication in this direction is mostly text-based. The ESP can send the following packet types:

### `STATUS` packet
This packet contains a single byte which indicates the status of the `ESP <-> server` connection. It can have the following values:
- `0x00` - ESP error or it is not detected
- `0x01` - the ESP is connecting to the specified WiFi network
- `0x02` - successfully connected to the WiFi network, but could not connect to the specified server
- `0x03` - successfully connected to the specified server

### `TEXT` packet
This packet contains ASCII text that is always terminated by a `'\n'`. Any `STATUS` characters should not be included in the text, but rather
interpreted as status codes.

## STM32 -> ESP8266
The communication in this direction is mostly binary due to performance reasons. Each packet starts with the packet type character, followed by the data,
and each packet is terminated by a `'\n'` character. The `'\n'` character should not appear anywhere else in the packet - this is achieved by
using an escape character: `'~'`. 

When the `'\n'` character appears somewhere in the packet, it should be represented by the following character sequence: `"~0"`, and the escape character
itself should be represented by the `"~1"` character sequence.

The STM32 can send the following packet types:
- `'C'` - config packet type, the data should be the ESP configuration text data represented in the following form: `"\tWIFI_SSID\tWIFI_PASSWORD\tSERVER_IP"`
- `'T'` - text packet type, the data should be a text string (for example a log from the STM32)
- `'D'` - data packet type, the data is in binary form representing a predefined *C struct* that contains all of the robot status and sensor information
that must be displayed on the telemetry page