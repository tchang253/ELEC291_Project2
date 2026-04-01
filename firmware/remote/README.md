# CV_IR_BRIDGE
## Remote Control Relay

This code runs on the **Silicon Labs EFM8 Laser Bee Development Kit**. It bridges the gap between the Python Computer Vision script running on your laptop and the STM32 robot on the floor.

### How it Works:
1. **Serial RX**: It listens on UART0 (`P0.4` TX, `P0.5` RX) for incoming ASCII direction commands (`'F'`, `'B'`, `'L'`, `'R'`, `'S'`).
2. **Translation**: It immediately translates these single-character commands into full proprietary 12-bit IR transmission packets.
3. **IR Output**: It toggles an infrared LED on pin `P2.1` using a 38kHz PWM carrier frequency (Timer 2) to blast the signal to the robot's receiver.
4. **LCD**: It visualizes the current moving state on the LCD screen, allowing for easy hardware-level debugging.

### Wiring the Bridge:
If your laptop connects via an FTDI USB-to-UART adapter rather than the built-in debugger:
* Connect **FTDI TXD** to **EFM8 `P0.5` (RX)**
* Connect **FTDI RXD** to **EFM8 `P0.4` (TX)**
* Connect **GND** to **GND**

### How to Compile:
Use the standard C51 Silicon Labs tools in Simplicity Studio or Keil to compile `CV_IR_BRIDGE.c` and flash it to the Laser Bee.
