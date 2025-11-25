# Makefile for ATmega32 quadcopter firmware (Bluetooth + OLED status)
# Usage:
#   make            -> build .hex
#   make flash      -> flash using avrdude (default programmer: usbasp)
#   make clean

MCU = atmega32
F_CPU = 16000000UL
CC = avr-gcc
OBJCOPY = avr-objcopy
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu11 -ffunction-sections -fdata-sections
LDFLAGS = -Wl,--gc-sections

SRC = quad_bt_oled.c
TARGET = quad
ELF = $(TARGET).elf
HEX = $(TARGET).hex

# Programmer and port for avrdude (edit if you use another programmer)
# Examples: prog=usbasp, prog=arduino, prog=avrisp
PROG ?= usbasp
PORT ?=

all: $(HEX)

$(ELF): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(ELF) $(LDFLAGS)

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex -R .eeprom $(ELF) $(HEX)

flash: $(HEX)
	avrdude -c $(PROG) -p m32 $(if $(PORT),-P $(PORT),) -U flash:w:$(HEX):i

clean:
	rm -f $(ELF) $(HEX)

.PHONY: all flash clean
