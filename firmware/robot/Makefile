SHELL=cmd
CC=arm-none-eabi-gcc
AS=arm-none-eabi-as
LD=arm-none-eabi-ld
CCFLAGS=-mcpu=cortex-m0 -mthumb -g 

# =====================================================
# Automatically locate the COM port if not specified
# (e.g. you can still override with: make flash PORT=COM4)
# =====================================================
PORT ?= $(shell powershell -noprofile -command "[System.IO.Ports.SerialPort]::getportnames() | Select-Object -First 1")
GCCPATH=$(subst \bin\arm-none-eabi-gcc.exe,\,$(shell where $(CC)))
LIBPATH1=$(subst \libgcc.a,,$(shell dir /s /b "$(GCCPATH)*libgcc.a" | find "v6-m"))
LIBPATH2=$(subst \libc_nano.a,,$(shell dir /s /b "$(GCCPATH)*libc_nano.a" | find "v6-m"))
LIBSPEC=-L"$(LIBPATH1)" -L"$(LIBPATH2)"

# =============================================================
# TARGET: main_cody  (PuTTY PID test - type fake ADC values)
# =============================================================
OBJS=main_cody.o motor.o error.o PIDCALC.o startup.o newlib_stubs.o serial.o

main_cody.elf: $(OBJS)
	$(LD) $(OBJS) $(LIBSPEC) -Os -nostdlib -lnosys -lgcc -T Common/LDscripts/stm32l051xx.ld --cref -Map main_cody.map -o main_cody.elf
	arm-none-eabi-objcopy -O ihex main_cody.elf main_cody.hex
	@echo Success!

main_cody.o: main_cody.c motor.h error.h PIDCALC.h
	$(CC) -c $(CCFLAGS) main_cody.c -o main_cody.o

# =============================================================
# SHARED OBJECT FILES
# =============================================================
motor.o: motor.c motor.h
	$(CC) -c $(CCFLAGS) motor.c -o motor.o

error.o: error.c error.h
	$(CC) -c $(CCFLAGS) error.c -o error.o

PIDCALC.o: PIDCALC.c PIDCALC.h
	$(CC) -c $(CCFLAGS) PIDCALC.c -o PIDCALC.o

startup.o: Common/Source/startup.c
	$(CC) -c $(CCFLAGS) -DUSE_USART1 Common/Source/startup.c -o startup.o

serial.o: Common/Source/serial.c
	$(CC) -c $(CCFLAGS) -DUSE_USART1 Common/Source/serial.c -o serial.o

newlib_stubs.o: Common/Source/newlib_stubs.c
	$(CC) -c $(CCFLAGS) Common/Source/newlib_stubs.c -o newlib_stubs.o

# =============================================================
# FLASH  (usage: make flash PORT=COM4)
# =============================================================
flash: main_cody.elf
	tools\stm32flash -w main_cody.hex -v -g 0x0 $(PORT)

# =============================================================
# CLEAN
# =============================================================
clean:
	@del *.o 2>NUL
	@del *.elf *.hex *.map *.lst 2>NUL
