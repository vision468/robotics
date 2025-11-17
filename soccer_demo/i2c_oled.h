#ifndef I2C_OLED_H
#define I2C_OLED_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* I2C Pins for ATmega32 */
#define OLED_SCL_PIN 0 // PORTC0
#define OLED_SDA_PIN 1 // PORTC1
#define OLED_PORT PORTC
#define OLED_DDR DDRC

#define OLED_ADDR 0x78 // OLED I2C 0x3C<<1

void oled_init();
void oled_clear();
void oled_print(const char *str);
void oled_update();

#endif
