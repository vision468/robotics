#include "i2c_oled.h"

/* -----------------------------
   Simple I2C bit-banging
------------------------------*/

#define I2C_DELAY() _delay_us(5)

void i2c_start()
{
    OLED_PORT |= (1 << OLED_SDA_PIN) | (1 << OLED_SCL_PIN);
    I2C_DELAY();
    OLED_PORT &= ~(1 << OLED_SDA_PIN);
    I2C_DELAY();
    OLED_PORT &= ~(1 << OLED_SCL_PIN);
}

void i2c_stop()
{
    OLED_PORT &= ~(1 << OLED_SDA_PIN);
    I2C_DELAY();
    OLED_PORT |= (1 << OLED_SCL_PIN);
    I2C_DELAY();
    OLED_PORT |= (1 << OLED_SDA_PIN);
    I2C_DELAY();
}

void i2c_write(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (data & 0x80)
            OLED_PORT |= (1 << OLED_SDA_PIN);
        else
            OLED_PORT &= ~(1 << OLED_SDA_PIN);

        I2C_DELAY();
        OLED_PORT |= (1 << OLED_SCL_PIN);
        I2C_DELAY();
        OLED_PORT &= ~(1 << OLED_SCL_PIN);
        data <<= 1;
    }
    // ACK pulse
    OLED_PORT |= (1 << OLED_SDA_PIN);
    I2C_DELAY();
    OLED_PORT |= (1 << OLED_SCL_PIN);
    I2C_DELAY();
    OLED_PORT &= ~(1 << OLED_SCL_PIN);
}

void oled_init()
{
    OLED_DDR |= (1 << OLED_SDA_PIN) | (1 << OLED_SCL_PIN);
    _delay_ms(100);
    // Initialization sequence for SSD1306 (simplified)
    i2c_start();
    i2c_write(OLED_ADDR); // address + write
    i2c_write(0x00);      // command
    i2c_write(0xAE);      // display off
    i2c_write(0xA6);      // normal display
    i2c_write(0xAF);      // display on
    i2c_stop();
}

void oled_clear()
{
    // Simple placeholder: just re-initialize
    oled_init();
}

void oled_print(const char *str)
{
    // Simple placeholder: not real font, just demo
    i2c_start();
    i2c_write(OLED_ADDR);
    i2c_write(0x40); // data
    while (*str)
    {
        i2c_write(*str);
        str++;
    }
    i2c_stop();
}

void oled_update()
{
    // In this simple version, print does immediate update
}
