# STM32 Robot Firmware — PuTTY PID Test

## Prerequisites
You need the ARM cross-compiler installed and on your PATH:
- **`arm-none-eabi-gcc`** — The GCC cross-compiler for ARM Cortex-M0
- Download: [GNU Arm Embedded Toolchain](https://developer.arm.com/downloads/-/gnu-rm)

To verify it's installed, open a terminal and run:
```
arm-none-eabi-gcc --version
```

## Quick Start

### 1. Compile
```bash
cd firmware/robot
make main_cody.elf
```

### 2. Flash
Find your COM port in **Device Manager > Ports (COM & LPT)**.
Then hold BOOT0, press RESET, release BOOT0, and run:
```bash
make flash PORT=COM4
```
(Replace `COM4` with your actual COM port)

### 3. Connect via PuTTY
- Connection type: **Serial**
- Serial line: **your COM port** (e.g. COM4)
- Speed: **115200**
- Hit RESET on the board (without holding BOOT0)

### 4. Use it
You'll see a `READY>` prompt. Type three comma-separated fake ADC values (e.g. `2000,1000,500`) and hit Enter. The PID will calculate motor speeds and spin the wheels.

## Wiring (USB-to-Serial Adapter to STM32)

| USB Adapter | STM32 Pin | Purpose |
|---|---|---|
| VCC / 3.3V | 3.3V | Power |
| GND | GND | Ground |
| TX | PA10 | Data to STM32 |
| RX | PA9 | Data from STM32 |

**Important:** Make sure your adapter's voltage jumper is set to **3.3V**, not 5V.

## Cleaning Build Files
```bash
make clean
```
