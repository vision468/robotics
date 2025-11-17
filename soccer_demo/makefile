# MCU name
MCU = atmega32

# Clock frequency
F_CPU = 8000000UL

# Compiler
CC = avr-gcc
OBJCOPY = avr-objcopy

# Source files
SOURCES = main.c i2c_oled.c

# Output file name (without extension)
TARGET = main

# Compiler flags
CFLAGS = -std=c11 -Wall -Os -DF_CPU=$(F_CPU) -mmcu=$(MCU)

# Build rules
all: $(TARGET).hex

$(TARGET).elf: $(SOURCES)
	$(CC) $(CFLAGS) -o $@ $(SOURCES)

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Clean build files
clean:
	rm -f $(TARGET).elf $(TARGET).hex
