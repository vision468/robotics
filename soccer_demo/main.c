/*
   Simple Educational Robot Football Code
   Target: ATmega32 - External Crystal
   Motors: 4 Mecanum wheels (controlled by 2×L298)
   Sensors:
       - 3× Ultrasonic (HC-SR04) with shared trigger
       - 8× IR ball sensors through 74HC4051 (analog to ADC0)
       - 4× Line sensors (digital)
   Display:
       - OLED (I2C)
   Controls:
       - Start Button
       - Menu1 (switch role: Attacker / Defender)
       - Menu2 (toggle debug info)
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "i2c_oled.h" // Include your OLED driver

#define F_CPU 8000000UL // 8MHz external crystal

/* -----------------------------
     ROBOT ROLES
------------------------------*/
#define ROLE_ATTACKER 0
#define ROLE_DEFENDER 1

/* -----------------------------
     GLOBAL VARIABLES
------------------------------*/
uint8_t robotRole = ROLE_ATTACKER;
uint8_t debugMode = 0;

/* -----------------------------
     PIN DEFINITIONS (Mapped for ATmega32)
------------------------------*/

/* Motor driver 1 (M1 & M2) */
#define M1_IN1 PD3
#define M1_IN2 PD2
#define M1_PWM PD4

#define M2_IN1 PB6
#define M2_IN2 PB7
#define M2_PWM PC3

/* Motor driver 2 (M3 & M4) */
#define M3_IN1 PC0
#define M3_IN2 PC1
#define M3_PWM PC2

#define M4_IN1 PC6
#define M4_IN2 PC7
#define M4_PWM PC4

/* Ultrasonic */
#define US_TRIG PD6
#define US_ECHO_L PD7
#define US_ECHO_F PD0
#define US_ECHO_R PD1

/* IR MUX */
#define MUX_A PC5
#define MUX_B PB4
#define MUX_C PB5
#define MUX_OUT_ADC 0 // ADC0

/* Line sensors */
#define LINE_F PA3
#define LINE_R PA2
#define LINE_B PA1
#define LINE_L PA0

/* Buttons */
#define BTN_START PD1
#define BTN_MENU1 PD5
#define BTN_MENU2 PD5 // mapped same pin (adjust if needed)

/* I2C pins (OLED) */
#define SDA_PIN PC3
#define SCL_PIN PC5

/* -------------------------------------------------------
      INITIAL SETUP FUNCTIONS
--------------------------------------------------------*/
void setup_io()
{
    // Motor pins as outputs
    DDRD |= (1 << M1_IN1) | (1 << M1_IN2) | (1 << M1_PWM) | (1 << US_TRIG);

    DDRB |= (1 << M2_IN1) | (1 << M2_IN2) | (1 << MUX_B) | (1 << MUX_C);
    DDRC |= (1 << M2_PWM) | (1 << M3_IN1) | (1 << M3_IN2) | (1 << M3_PWM) | (1 << M4_IN1) | (1 << M4_IN2) | (1 << M4_PWM) | (1 << MUX_A);

    // Ultrasonic echo as input
    DDRD &= ~((1 << US_ECHO_L) | (1 << US_ECHO_F) | (1 << US_ECHO_R));

    // Line sensors as input
    DDRA &= ~((1 << LINE_F) | (1 << LINE_R) | (1 << LINE_B) | (1 << LINE_L));

    // Buttons as input
    DDRD &= ~((1 << BTN_START) | (1 << BTN_MENU1) | (1 << BTN_MENU2));
}

/* -------------------------------------------------------
      ADC INITIALIZATION
--------------------------------------------------------*/
void adc_init()
{
    ADMUX = (1 << REFS0);     // AVCC reference
    ADCSRA = (1 << ADEN) | 7; // Enable ADC, prescaler 128
}

uint16_t adc_read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0) | channel;
    _delay_us(10);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
        ;
    return ADC;
}

/* -------------------------------------------------------
      MUX FUNCTIONS (8 IR sensors)
--------------------------------------------------------*/
uint16_t read_IR_sensor(uint8_t n)
{
    if (n > 7)
        return 0;

    // Select sensor
    PORTC = (PORTC & 0b11000000) | ((n & 1) << MUX_A);

    PORTB = (PORTB & 0b00001111) | (((n >> 1) & 1) << MUX_B) | (((n >> 2) & 1) << MUX_C);

    _delay_us(50);

    return adc_read(MUX_OUT_ADC);
}

/* -------------------------------------------------------
      LINE SENSOR READ
--------------------------------------------------------*/
uint8_t read_line_front() { return (PINA & (1 << LINE_F)) ? 1 : 0; }
uint8_t read_line_right() { return (PINA & (1 << LINE_R)) ? 1 : 0; }
uint8_t read_line_back() { return (PINA & (1 << LINE_B)) ? 1 : 0; }
uint8_t read_line_left() { return (PINA & (1 << LINE_L)) ? 1 : 0; }

/* -------------------------------------------------------
      BUTTON HANDLING
--------------------------------------------------------*/
uint8_t button_pressed(uint8_t pin)
{
    if (!(PIND & (1 << pin))) // active LOW
    {
        _delay_ms(20);
        if (!(PIND & (1 << pin)))
            return 1;
    }
    return 0;
}

/* -------------------------------------------------------
      MOTOR CONTROL (simple)
--------------------------------------------------------*/
void motor_stop()
{
    PORTD &= ~((1 << M1_IN1) | (1 << M1_IN2));
    PORTB &= ~((1 << M2_IN1) | (1 << M2_IN2));
    PORTC &= ~((1 << M2_PWM) | (1 << M3_IN1) | (1 << M3_IN2) | (1 << M3_PWM) | (1 << M4_IN1) | (1 << M4_IN2) | (1 << M4_PWM));
}

void motor_forward_simple()
{
    PORTD |= (1 << M1_IN1);
    PORTD &= ~(1 << M1_IN2);

    PORTB |= (1 << M2_IN1);
    PORTB &= ~(1 << M2_IN2);

    PORTC |= (1 << M3_IN1);
    PORTC &= ~(1 << M3_IN2);

    PORTC |= (1 << M4_IN1);
    PORTC &= ~(1 << M4_IN2);
}

/* -------------------------------------------------------
      BALL DIRECTION (simple heuristic)
--------------------------------------------------------*/
uint8_t get_ball_direction()
{
    uint16_t values[8];
    uint8_t i, best = 0;

    for (i = 0; i < 8; i++)
    {
        values[i] = read_IR_sensor(i);
        if (values[i] > values[best])
            best = i;
    }
    return best; // sector number 0..7
}

/* -------------------------------------------------------
      ULTRASONIC SIMPLE MEASUREMENT
--------------------------------------------------------*/
uint16_t measure_ultrasonic(uint8_t echoPin)
{
    uint16_t time = 0;

    PORTD |= (1 << US_TRIG);
    _delay_us(10);
    PORTD &= ~(1 << US_TRIG);

    // wait for rising
    while (!(PIND & (1 << echoPin)))
        ;

    while (PIND & (1 << echoPin))
    {
        time++;
        _delay_us(1);
    }

    return time / 58; // approximate cm
}

/* -------------------------------------------------------
      ROLE SWITCHING
--------------------------------------------------------*/
void check_role_buttons()
{
    if (button_pressed(BTN_MENU1))
    {
        if (robotRole == ROLE_ATTACKER)
            robotRole = ROLE_DEFENDER;
        else
            robotRole = ROLE_ATTACKER;
    }

    if (button_pressed(BTN_MENU2))
    {
        debugMode = !debugMode;
    }
}

/* -------------------------------------------------------
      SIMPLE BEHAVIOR (STATE MACHINE)
--------------------------------------------------------*/
void behavior_attacker()
{
    uint8_t ball = get_ball_direction();

    if (read_line_front())
    {
        motor_stop();
        _delay_ms(200);
        return;
    }

    if (ball < 3 || ball > 5)
        motor_forward_simple();
    else
        motor_stop();
}

void behavior_defender()
{
    uint16_t distFront = measure_ultrasonic(US_ECHO_F);

    if (distFront < 20)
    {
        motor_stop();
        return;
    }

    motor_forward_simple();
}

/* -------------------------------------------------------
      MAIN LOOP
--------------------------------------------------------*/
int main()
{
    setup_io();
    adc_init();
    oled_init(); // initialize OLED
    sei();

    while (1)
    {
        check_role_buttons();

        // Update OLED
        oled_clear();
        if (robotRole == ROLE_ATTACKER)
            oled_print("Role: Attacker");
        else
            oled_print("Role: Defender");

        if (debugMode)
        {
            oled_print("Debug: ON");
        }
        else
        {
            oled_print("Debug: OFF");
        }
        oled_update();

        if (button_pressed(BTN_START))
        {
            motor_stop();
            _delay_ms(300);
        }

        if (robotRole == ROLE_ATTACKER)
            behavior_attacker();
        else
            behavior_defender();
    }

    return 0;
}
