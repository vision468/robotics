/*
 * quad_bt_oled.c
 *
 * ATmega32 firmware (assumes 16MHz clock)
 * - Bluetooth via UART (PD0 RX, PD1 TX)
 * - MPU6050 via TWI (hardware TWI pins)
 * - OLED SSD1306 (I2C address 0x3C) for status display (128x32)
 * - Motors: m1 -> PB3 (soft), m2 -> PD5 (OC1A), m3 -> PD4 (OC1B), m4 -> PD7 (soft)
 * - Timer1 used for hardware PWM (50Hz) for m2/m3
 * - Timer0 CTC 1ms tick for frame mgmt & software PWM for m1/m4
 *
 * Keep code readable and commented for ease of tuning.
 *
 * Safety: read comments at file bottom before testing with props.
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* -------------------- Config -------------------- */
#define BAUD 115200
#define UBRR_VAL ((F_CPU / 16 / BAUD) - 1)

#define MPU_ADDR 0x68
#define SSD1306_ADDR 0x3C

/* Motor pins */
#define M1_PORT PORTB
#define M1_DDR DDRB
#define M1_PINB PINB
#define M1_PIN PB3 // software motor

#define M4_PORT PORTD
#define M4_DDR DDRD
#define M4_PIN PD7 // software motor

/* Timing */
volatile uint16_t frame_ms = 0; // 0..19 (20ms frame)
volatile uint32_t sys_ms = 0;

/* Motor pulse widths (microseconds) */
volatile uint16_t motor_us[4] = {1000, 1000, 1000, 1000};

/* software pwm states */
volatile uint8_t sw_active[2] = {0, 0};
volatile uint16_t sw_end_ms[2] = {0, 0};

/* Flight state */
volatile uint8_t armed = 0;
volatile uint16_t throttle = 1000; // 1000..2000 from BT

/* IMU data */
volatile float accel_x, accel_y, accel_z;
volatile float gyro_x, gyro_y, gyro_z;
volatile float roll = 0.0f, pitch = 0.0f;

/* PID structures */
typedef struct
{
    float kp, ki, kd;
    float prev;
    float integ;
    float out_min, out_max;
} pid_t;

pid_t pid_roll = {4.0f, 0.01f, 0.8f, 0, 0, -400, 400};
pid_t pid_pitch = {4.0f, 0.01f, 0.8f, 0, 0, -400, 400};

volatile float set_roll = 0.0f, set_pitch = 0.0f;

/* -------------------- Utility: UART -------------------- */
void uart_init(void)
{
    UBRRH = (uint8_t)(UBRR_VAL >> 8);
    UBRRL = (uint8_t)(UBRR_VAL);
    UCSRB = (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void uart_putc(char c)
{
    while (!(UCSRA & (1 << UDRE)))
        ;
    UDR = c;
}
void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}
char uart_getc_nb(void)
{
    if (UCSRA & (1 << RXC))
        return UDR;
    return 0;
}

/* -------------------- I2C / TWI (basic blocking) -------------------- */
void twi_init(void)
{
    TWSR = 0x00; // prescaler = 1
    TWBR = 32;   // ~100kHz at 16MHz
}

uint8_t twi_start_addr_rw(uint8_t addr_rw)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    TWDR = addr_rw;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    return (TWSR & 0xF8);
}

void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    _delay_us(10);
}

uint8_t twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    return (TWSR & 0xF8);
}

uint8_t twi_read_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)))
        ;
    return TWDR;
}

uint8_t twi_read_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    return TWDR;
}

/* helper: write reg to device */
void i2c_write_reg(uint8_t dev, uint8_t reg, uint8_t val)
{
    twi_start_addr_rw((dev << 1) | 0);
    twi_write(reg);
    twi_write(val);
    twi_stop();
}

/* helper: read multiple bytes from device starting at reg */
void i2c_read_bytes(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    twi_start_addr_rw((dev << 1) | 0);
    twi_write(reg);
    twi_start_addr_rw((dev << 1) | 1);
    for (uint8_t i = 0; i < len; i++)
    {
        if (i < len - 1)
            buf[i] = twi_read_ack();
        else
            buf[i] = twi_read_nack();
    }
    twi_stop();
}

/* -------------------- SSD1306 (128x32) minimal driver -------------------- */
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 32
#define SSD1306_PAGES (SSD1306_HEIGHT / 8)
static uint8_t ssd_buf[SSD1306_WIDTH * SSD1306_PAGES];

void ssd_cmd(uint8_t c)
{
    twi_start_addr_rw((SSD1306_ADDR << 1) | 0);
    twi_write(0x00); // control: command
    twi_write(c);
    twi_stop();
}

void ssd_data_start(void)
{
    twi_start_addr_rw((SSD1306_ADDR << 1) | 0);
    twi_write(0x40); // control: data
}

void ssd_data_write(uint8_t d)
{
    twi_write(d);
}

void ssd_init(void)
{
    _delay_ms(100);
    ssd_cmd(0xAE); // display off
    ssd_cmd(0x20);
    ssd_cmd(0x00); // memory addr mode horizontal
    ssd_cmd(0xB0); // page start 0
    ssd_cmd(0xC8); // COM scan dec
    ssd_cmd(0x00); // low col
    ssd_cmd(0x10); // high col
    ssd_cmd(0x40); // start line
    ssd_cmd(0x81);
    ssd_cmd(0x7F); // contrast
    ssd_cmd(0xA1); // seg remap
    ssd_cmd(0xA6); // normal display
    ssd_cmd(0xA8);
    ssd_cmd(0x1F); // multiplex 32
    ssd_cmd(0xA4); // display follow RAM
    ssd_cmd(0xD3);
    ssd_cmd(0x00); // display offset
    ssd_cmd(0xD5);
    ssd_cmd(0x80); // osc freq
    ssd_cmd(0xD9);
    ssd_cmd(0xF1); // pre-charge
    ssd_cmd(0xDA);
    ssd_cmd(0x02); // com pins
    ssd_cmd(0xDB);
    ssd_cmd(0x40); // vcom detect
    ssd_cmd(0x8D);
    ssd_cmd(0x14); // charge pump
    ssd_cmd(0xAF); // display on
    // clear buffer
    memset(ssd_buf, 0, sizeof(ssd_buf));
}

void ssd_update(void)
{
    for (uint8_t page = 0; page < SSD1306_PAGES; page++)
    {
        ssd_cmd(0xB0 + page);
        ssd_cmd(0x00);
        ssd_cmd(0x10);
        ssd_data_start();
        for (uint8_t col = 0; col < SSD1306_WIDTH; col++)
        {
            ssd_data_write(ssd_buf[page * SSD1306_WIDTH + col]);
        }
        twi_stop();
    }
}

/* Tiny 5x7 font (only basic ASCII 32..127). For brevity include minimal subset (digits, letters, few symbols).
   To keep file compact, implement a minimal numeric/alpha+symbols renderer. */
static const uint8_t font5x7[][5] = {
    /* space (32) .. 47 */ {0}, {0}, {0}, {0}, {0} // placeholder for simplicity
    // For readability and brevity, we'll provide a very small glyph function below
};

void draw_char_simple(int x, int page, char c)
{
    // Very small renderer: render digits, minus, dot, letters A-Z limited
    // Each character 5x7 -> occupies 6 columns with 1px spacing
    uint8_t coldata[6];
    memset(coldata, 0, 6);
    if (c >= '0' && c <= '9')
    {
        // crude patterns for digits 0-9 (6x8 columns)
        static const uint8_t digits[10][6] = {
            {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00}, // 0
            {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00}, // 1
            {0x42, 0x61, 0x51, 0x49, 0x46, 0x00}, // 2
            {0x21, 0x41, 0x45, 0x4B, 0x31, 0x00}, // 3
            {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00}, // 4
            {0x27, 0x45, 0x45, 0x45, 0x39, 0x00}, // 5
            {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00}, // 6
            {0x01, 0x71, 0x09, 0x05, 0x03, 0x00}, // 7
            {0x36, 0x49, 0x49, 0x49, 0x36, 0x00}, // 8
            {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00}  // 9
        };
        memcpy(coldata, digits[c - '0'], 6);
    }
    else if (c == '-')
    {
        coldata[2] = 0x08;
    }
    else if (c == '.')
    {
        coldata[0] = 0x80;
    }
    else if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
    {
        // crude: show first letter as single column pattern to indicate letter (not perfect)
        coldata[0] = 0x7E;
        coldata[1] = 0x09;
        coldata[2] = 0x09;
        coldata[3] = 0x09;
        coldata[4] = 0x7E;
    }
    else
    {
        // unknown -> blank
    }
    // copy into buffer at given x and page
    for (uint8_t i = 0; i < 6; i++)
    {
        int col = x + i;
        if (col < 0 || col >= SSD1306_WIDTH)
            continue;
        ssd_buf[page * SSD1306_WIDTH + col] = coldata[i];
    }
}

void draw_text(int x, int page, const char *s)
{
    while (*s)
    {
        draw_char_simple(x, page, *s++);
        x += 6;
        if (x >= SSD1306_WIDTH)
            break;
    }
}

/* -------------------- MPU6050 helpers -------------------- */
void mpu_init(void)
{
    // wake device
    i2c_write_reg(MPU_ADDR, 0x6B, 0x00); // PWR_MGMT_1 = 0
    // accel ±2g
    i2c_write_reg(MPU_ADDR, 0x1C, 0x00);
    // gyro ±250
    i2c_write_reg(MPU_ADDR, 0x1B, 0x00);
    // low pass filter
    i2c_write_reg(MPU_ADDR, 0x1A, 0x03);
    _delay_ms(50);
}

void mpu_read_all(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];
    i2c_read_bytes(MPU_ADDR, 0x3B, buf, 14);
    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *gx = (int16_t)((buf[8] << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
}

void mpu_update_floats(void)
{
    int16_t ax, ay, az, gx, gy, gz;
    mpu_read_all(&ax, &ay, &az, &gx, &gy, &gz);
    accel_x = (float)ax / 16384.0f;
    accel_y = (float)ay / 16384.0f;
    accel_z = (float)az / 16384.0f;
    gyro_x = (float)gx / 131.0f;
    gyro_y = (float)gy / 131.0f;
    gyro_z = (float)gz / 131.0f;
}

/* complementary filter update: call at dt seconds */
void complementary_update(float dt)
{
    float acc_roll = atan2f(accel_y, accel_z) * 57.2957795f;
    float acc_pitch = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * 57.2957795f;
    const float alpha = 0.98f;
    roll = alpha * (roll + gyro_x * dt) + (1.0f - alpha) * acc_roll;
    pitch = alpha * (pitch + gyro_y * dt) + (1.0f - alpha) * acc_pitch;
}

/* -------------------- Timer and PWM setup -------------------- */
void timers_init(void)
{
    cli();
    // Timer1: Fast PWM, ICR1 top -> 20ms period
    // prescaler = 8 -> tick = 0.5us; TOP = 40000 -> 20ms
    TCCR1A = (1 << COM1A1) | (1 << COM1B1);             // non-inverting OC1A and OC1B
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // mode 14 (fast PWM w/ ICR1)
    ICR1 = 40000;
    // initial compare values
    OCR1A = (uint16_t)(motor_us[1] * 2); // m2
    OCR1B = (uint16_t)(motor_us[2] * 2); // m3
    DDRD |= (1 << PD5) | (1 << PD4);     // OC1A/OC1B outputs

    // Timer0 CTC for ~1ms tick (prescaler 64 -> tick 4us -> OCR0=249)
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);
    OCR0 = 249;
    TIMSK |= (1 << OCIE0); // enable compare match

    // software motor pins
    M1_DDR |= (1 << M1_PIN);
    M4_DDR |= (1 << M4_PIN);

    sei();
}

/* Timer0 Compare ISR: ~1ms tick */
ISR(TIMER0_COMP_vect)
{
    frame_ms++;
    sys_ms++;
    if (frame_ms >= 20)
    {
        frame_ms = 0;
        // start of new frame: raise software motor pins
        if (motor_us[0] > 0)
        {
            sw_active[0] = 1;
            M1_PORT |= (1 << M1_PIN);
            uint16_t end = (motor_us[0] + 999) / 1000;
            if (end == 0)
                end = 1;
            sw_end_ms[0] = end;
        }
        else
        {
            sw_active[0] = 0;
            M1_PORT &= ~(1 << M1_PIN);
        }
        if (motor_us[3] > 0)
        {
            sw_active[1] = 1;
            M4_PORT |= (1 << M4_PIN);
            uint16_t end = (motor_us[3] + 999) / 1000;
            if (end == 0)
                end = 1;
            sw_end_ms[1] = end;
        }
        else
        {
            sw_active[1] = 0;
            M4_PORT &= ~(1 << M4_PIN);
        }
    }
    else
    {
        if (sw_active[0])
        {
            if (frame_ms >= sw_end_ms[0])
            {
                M1_PORT &= ~(1 << M1_PIN);
                sw_active[0] = 0;
            }
        }
        if (sw_active[1])
        {
            if (frame_ms >= sw_end_ms[1])
            {
                M4_PORT &= ~(1 << M4_PIN);
                sw_active[1] = 0;
            }
        }
    }
}

/* -------------------- Motor helpers -------------------- */
void set_motor_us(uint8_t idx, uint16_t us)
{
    if (us < 1000)
        us = 1000;
    if (us > 2000)
        us = 2000;
    motor_us[idx] = us;
    if (idx == 1)
        OCR1A = (uint16_t)(us * 2);
    if (idx == 2)
        OCR1B = (uint16_t)(us * 2);
}

/* -------------------- PID compute -------------------- */
int16_t pid_compute(pid_t *pid, float setpoint, float measured, float dt_ms)
{
    float err = setpoint - measured;
    pid->integ += err * (dt_ms / 1000.0f);
    float deriv = 0.0f;
    static float last_err_roll = 0.0f; // not ideal but simple
    deriv = (err - pid->prev) / (dt_ms / 1000.0f);
    pid->prev = err;
    float out = pid->kp * err + pid->ki * pid->integ + pid->kd * deriv;
    if (out > pid->out_max)
        out = pid->out_max;
    if (out < pid->out_min)
        out = pid->out_min;
    return (int16_t)out;
}

/* -------------------- Bluetooth command parser (simple ASCII) -------------------- */
#define BT_MAX 64
char bt_line[BT_MAX];
uint8_t bt_idx = 0;

void process_bt_line(char *ln)
{
    // Commands:
    // ARM
    // DIS
    // THR:1000..2000
    // ROLL:-30..30
    // PITCH:-30..30
    // TUNE:Rkp,ki,kd (optional)
    if (strncmp(ln, "ARM", 3) == 0)
    {
        armed = 1;
        uart_puts("ARMED\n");
    }
    else if (strncmp(ln, "DIS", 3) == 0)
    {
        armed = 0;
        set_motor_us(0, 1000);
        set_motor_us(1, 1000);
        set_motor_us(2, 1000);
        set_motor_us(3, 1000);
        uart_puts("DISARMED\n");
    }
    else if (strncmp(ln, "THR:", 4) == 0)
    {
        int v = atoi(ln + 4);
        if (v < 1000)
            v = 1000;
        if (v > 2000)
            v = 2000;
        throttle = v;
        uart_puts("THR_OK\n");
    }
    else if (strncmp(ln, "ROLL:", 5) == 0)
    {
        set_roll = atof(ln + 5);
        uart_puts("ROLL_OK\n");
    }
    else if (strncmp(ln, "PITCH:", 6) == 0)
    {
        set_pitch = atof(ln + 6);
        uart_puts("PITCH_OK\n");
    }
    else if (strncmp(ln, "TUNE:R", 6) == 0)
    {
        float a, b, c;
        if (sscanf(ln + 6, "%f,%f,%f", &a, &b, &c) == 3)
        {
            pid_roll.kp = a;
            pid_roll.ki = b;
            pid_roll.kd = c;
            pid_pitch.kp = a;
            pid_pitch.ki = b;
            pid_pitch.kd = c;
            uart_puts("TUNED\n");
        }
        else
            uart_puts("TUNE_ERR\n");
    }
    else
    {
        uart_puts("UNK\n");
    }
}

/* -------------------- Main -------------------- */
int main(void)
{
    // init peripherals
    uart_init();
    twi_init();
    mpu_init();
    ssd_init();
    timers_init();

    // ensure motors off initially
    M1_PORT &= ~(1 << M1_PIN);
    M4_PORT &= ~(1 << M4_PIN);
    OCR1A = (uint16_t)(motor_us[1] * 2);
    OCR1B = (uint16_t)(motor_us[2] * 2);

    uart_puts("FCU ready\n");

    uint32_t last_ctrl_ms = 0;
    const uint16_t ctrl_dt = 10; // ms
    uint32_t last_oled = 0;

    // main loop
    while (1)
    {
        // non-blocking UART read and line assembly
        char c = uart_getc_nb();
        if (c)
        {
            if (c == '\r')
            { /* ignore */
            }
            else if (c == '\n' || bt_idx >= BT_MAX - 1)
            {
                bt_line[bt_idx] = 0;
                if (bt_idx > 0)
                    process_bt_line(bt_line);
                bt_idx = 0;
            }
            else
            {
                bt_line[bt_idx++] = c;
            }
        }

        // control loop every ctrl_dt ms (approx)
        if ((sys_ms - last_ctrl_ms) >= ctrl_dt)
        {
            last_ctrl_ms = sys_ms;
            // read IMU
            mpu_update_floats();
            complementary_update(ctrl_dt / 1000.0f);

            // compute PID outputs
            int16_t out_roll = pid_compute(&pid_roll, set_roll, roll, ctrl_dt);
            int16_t out_pitch = pid_compute(&pid_pitch, set_pitch, pitch, ctrl_dt);

            // basic mixer (X configuration)
            int32_t m1 = (int32_t)throttle + out_pitch + out_roll;
            int32_t m2 = (int32_t)throttle + out_pitch - out_roll;
            int32_t m3 = (int32_t)throttle - out_pitch - out_roll;
            int32_t m4 = (int32_t)throttle - out_pitch + out_roll;

            if (!armed)
            {
                m1 = m2 = m3 = m4 = 1000;
            }

            // clamp and apply
            if (m1 < 1000)
                m1 = 1000;
            if (m1 > 2000)
                m1 = 2000;
            if (m2 < 1000)
                m2 = 1000;
            if (m2 > 2000)
                m2 = 2000;
            if (m3 < 1000)
                m3 = 1000;
            if (m3 > 2000)
                m3 = 2000;
            if (m4 < 1000)
                m4 = 1000;
            if (m4 > 2000)
                m4 = 2000;

            set_motor_us(0, (uint16_t)m1);
            set_motor_us(1, (uint16_t)m2);
            set_motor_us(2, (uint16_t)m3);
            set_motor_us(3, (uint16_t)m4);
        }

        // update OLED every 200ms
        if ((sys_ms - last_oled) >= 200)
        {
            last_oled = sys_ms;
            // prepare screen
            memset(ssd_buf, 0, sizeof(ssd_buf));
            char line[32];
            // Line 0: Armed/Disarmed and Throttle
            if (armed)
                draw_text(0, 0, "ARMED");
            else
                draw_text(0, 0, "DISARM");
            snprintf(line, sizeof(line), "T:%u", (unsigned)throttle);
            draw_text(64, 0, line);
            // Line 1: Roll/Pitch values
            snprintf(line, sizeof(line), "R:%.1f P:%.1f", roll, pitch);
            draw_text(0, 1, line);
            // Line 2: motor pulses
            snprintf(line, sizeof(line), "M1:%u M2:%u", (unsigned)motor_us[0], (unsigned)motor_us[1]);
            draw_text(0, 2, line);
            snprintf(line, sizeof(line), "M3:%u M4:%u", (unsigned)motor_us[2], (unsigned)motor_us[3]);
            draw_text(0, 3, line);
            ssd_update();
        }
    }

    return 0;
}

/* ------------- Notes & Safety -------------
 - Wiring summary:
   * MPU-6050: SDA->PC1, SCL->PC0 (TWI hardware pins on ATmega32)
   * SSD1306: SDA->PC1, SCL->PC0 (shared I2C bus, address 0x3C)
   * Bluetooth (serial): RX -> PD0, TX -> PD1 (use HC-05/06 or similar)
   * Motors:
       - M1 (soft) -> PB3
       - M2 (OC1A)  -> PD5
       - M3 (OC1B)  -> PD4
       - M4 (soft) -> PD7
   * Ensure common GND between battery/ESC and MCU.
 - ESC calibration:
   * Many ESCs expect 2000us throttle at power-on to set max, consult ESC manual.
 - Before installing props:
   * Keep props removed. Test arming/disarming and small throttle values.
 - Timing precision:
   * Two channels (m2,m3) are hardware PWM (high precision).
   * Two channels (m1,m4) are software-driven at 1ms granularity (~±1ms).
     If you need finer resolution, replace Timer0 tick with 250us or use another MCU with more HW PWM channels.
 - PID tuning:
   * Default gains are conservative starting points. Tune on a safe test rig.
 - Flashing:
   * Build: `make`
   * Flash (example): `make PROG=usbasp flash`
-------------------------------------------- */
