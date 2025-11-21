#include <Arduino.h>
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

// ---------------- Pin Defines ---------------- //
#define LIFT_FAN PD4
#define FAN_THRUST_PIN PD6  // OC0A

#define TRIG_PIN_LEFT PB3
#define ECHO_PIN_LEFT PD2   // INT0
#define TRIG_PIN_RIGHT PB5
#define ECHO_PIN_RIGHT PD3  // INT1

#define SERVO_PIN PB1       // Software PWM

#define IR_PIN 0            // ADC0 (PC0)

#define IMU_ADDR 0x68

// ---------------- IMU Variables ---------------- //
float gyro_scale = 65.5f;
float acc_scale = 8192.0f;

int16_t AccX, AccY, AccZ, GyrZ;
float accX, accY, accZ, gyrZ;
float yaw = 0.0f;
float offset_ax, offset_ay, offset_az, offset_gz;

unsigned long last_imu_time = 0;

// ---------------- Ultrasonic Variables (Timer1) ---------------- //
volatile uint16_t pulse_start_left = 0;
volatile uint16_t pulse_width_left = 0;
volatile uint8_t echo_state_left = 0;

volatile uint16_t pulse_start_right = 0;
volatile uint16_t pulse_width_right = 0;
volatile uint8_t echo_state_right = 0;

uint32_t left_distance = 0;
uint32_t right_distance = 0;

// ---------------- IR Variable ---------------- //
uint16_t ir_distance = 0;

// ---------------- Servo Variables (Timer2 software PWM) ---------------- //
volatile uint16_t servo_pulse_width = 1500;  // microseconds (1000-2000)
volatile uint16_t servo_timer = 0;           // millisecond counter for 20ms frame

// ---------------- Timing Variable ---------------- //
volatile unsigned long milliseconds = 0;

// ---------------- UART Functions ---------------- //
void uartTransmit(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uartPrint(const char* str) {
    while (*str) {
        uartTransmit(*str++);
    }
}

void uartPrintInt(uint32_t num) {
    char buffer[10];
    itoa(num, buffer, 10);
    uartPrint(buffer);
}

void uartPrintFloat(float num) {
    char buffer[20];
    dtostrf(num, 6, 2, buffer);
    uartPrint(buffer);
}

// ---------------- I2C Functions ---------------- //
void i2c_init(void) {
    TWSR = 0x00;
    TWBR = 12;
}

void i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void) {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    _delay_us(100);
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t i2c_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t i2c_read_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// ---------------- ADC Functions ---------------- //
void ADC_init() {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t readADC(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADCW;
}

uint16_t readIR() {
    uint16_t sensorValue = readADC(IR_PIN);
    if (sensorValue > 3) {
        float distance = (6787.0 / (sensorValue - 3.0)) - 4.0;
        if (distance >= 0 && distance <= 200) {
            return (uint16_t)distance;
        }
    }
    return 9999;
}

// ---------------- Ultrasonic Interrupts (Timer1 for timing) ---------------- //
ISR(INT0_vect) {
    if (PIND & (1 << ECHO_PIN_LEFT)) {
        pulse_start_left = TCNT1;
        echo_state_left = 1;
    } else {
        if (echo_state_left == 1) {
            uint16_t end = TCNT1;
            if (end >= pulse_start_left) {
                pulse_width_left = end - pulse_start_left;
            } else {
                pulse_width_left = (0xFFFF - pulse_start_left) + end;
            }
            echo_state_left = 0;
        }
    }
}

ISR(INT1_vect) {
    if (PIND & (1 << ECHO_PIN_RIGHT)) {
        pulse_start_right = TCNT1;
        echo_state_right = 1;
    } else {
        if (echo_state_right == 1) {
            uint16_t end = TCNT1;
            if (end >= pulse_start_right) {
                pulse_width_right = end - pulse_start_right;
            } else {
                pulse_width_right = (0xFFFF - pulse_start_right) + end;
            }
            echo_state_right = 0;
        }
    }
}

void triggerUS_Left() {
    PORTB &= ~(1 << TRIG_PIN_LEFT);
    _delay_us(2);
    PORTB |= (1 << TRIG_PIN_LEFT);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN_LEFT);
}

void triggerUS_Right() {
    PORTB &= ~(1 << TRIG_PIN_RIGHT);
    _delay_us(2);
    PORTB |= (1 << TRIG_PIN_RIGHT);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN_RIGHT);
}

uint32_t getDistance_Left() {
    if (pulse_width_left == 0 || pulse_width_left > 50000) {
        return 9999;
    }
    float pulse_us = pulse_width_left * 0.5;  // Timer1 prescaler=8, tick=0.5us
    return (uint32_t)(pulse_us / 58.0);
}

uint32_t getDistance_Right() {
    if (pulse_width_right == 0 || pulse_width_right > 50000) {
        return 9999;
    }
    float pulse_us = pulse_width_right * 0.5;
    return (uint32_t)(pulse_us / 58.0);
}

// ---------------- Timer2: millis() + Servo Software PWM ---------------- //
ISR(TIMER2_COMPA_vect) {
    milliseconds++;
    
    // Software PWM for servo (20ms period)
    servo_timer++;
    
    if (servo_timer == 1) {
        // Start pulse
        PORTB |= (1 << SERVO_PIN);
    }
    
    // End pulse when we reach the desired pulse width (in milliseconds)
    if (servo_timer == (servo_pulse_width / 1000)) {
        PORTB &= ~(1 << SERVO_PIN);
    }
    
    // Reset every 20ms
    if (servo_timer >= 20) {
        servo_timer = 0;
    }
}

unsigned long millis() {
    unsigned long m;
    cli();
    m = milliseconds;
    sei();
    return m;
}

// ---------------- Servo Control ---------------- //
void set_servo_angle(float angle) {
    if (angle < -90) angle = -90;
    if (angle > 90) angle = 90;
    
    // Servo: 1000us = -90°, 1500us = 0°, 2000us = +90°
    servo_pulse_width = 1500 + (int16_t)(angle * 500.0 / 90.0);
    
    uartPrint("  > Servo pulse width: ");
    uartPrintInt(servo_pulse_width);
    uartPrint(" us\r\n");
}

// ---------------- Fan Control ---------------- //
void startLiftFan() {
    PORTD |= (1 << LIFT_FAN);
    uartPrint("  > Lift fan ON (PD4 HIGH)\r\n");
}

void stopLiftFan() {
    PORTD &= ~(1 << LIFT_FAN);
    uartPrint("  > Lift fan OFF (PD4 LOW)\r\n");
}

void setThrustFan(uint8_t speed) {
    OCR0A = speed;
    uartPrint("  > Thrust fan PWM: ");
    uartPrintInt(speed);
    uartPrint("/255\r\n");
}

// ---------------- IMU Functions ---------------- //
void mpu6050_init(void) {
    uartPrint("Initializing MPU6050...\r\n");
    
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x6B);
    i2c_write(0x00);
    i2c_stop();
    _delay_ms(10);

    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1B);
    i2c_write(1 << 3);
    i2c_stop();

    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1C);
    i2c_write(0x08);
    i2c_stop();
    
    uartPrint("MPU6050 initialized\r\n");
}

void readIMURaw() {
    uint8_t data[14];

    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x3B);
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 1);
    for (uint8_t i = 0; i < 13; i++) data[i] = i2c_read_ack();
    data[13] = i2c_read_nack();
    i2c_stop();

    AccX = ((int16_t)data[0] << 8) | data[1];
    AccY = ((int16_t)data[2] << 8) | data[3];
    AccZ = ((int16_t)data[4] << 8) | data[5];
    GyrZ = ((int16_t)data[12] << 8) | data[13];

    accX = (float)AccX / acc_scale;
    accY = (float)AccY / acc_scale;
    accZ = (float)AccZ / acc_scale;
    gyrZ = (float)GyrZ / gyro_scale;
}

void calibrateIMU() {
    uartPrint("Calibrating IMU...\r\n");
    
    float sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gz = 0;
    
    for (int i = 0; i < 1000; i++) {
        readIMURaw();
        if (i >= 100) {
            sum_ax += accX;
            sum_ay += accY;
            sum_az += accZ;
            sum_gz += gyrZ;
        }
        _delay_ms(2);
    }
    
    offset_ax = sum_ax / 900.0f;
    offset_ay = sum_ay / 900.0f;
    offset_az = (sum_az / 900.0f) - 1.0f;
    offset_gz = sum_gz / 900.0f;
    
    uartPrint("Calibration complete!\r\n");
}

void readIMU() {
    readIMURaw();
    accX -= offset_ax;
    accY -= offset_ay;
    accZ -= offset_az;
    gyrZ -= offset_gz;
}

// ---------------- Setup ---------------- //
void setup() {
    // UART - 9600 baud
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    // ===== LIFT FAN (PD4) - Digital output ===== //
    DDRD |= (1 << LIFT_FAN);
    PORTD &= ~(1 << LIFT_FAN);

    // ===== THRUST FAN (PD6 / OC0A) - Timer0 Fast PWM ===== //
    DDRD |= (1 << FAN_THRUST_PIN);
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);  // Fast PWM, non-inverting
    TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler 64 → ~976Hz PWM
    OCR0A = 0;

    // ===== SERVO (PB1) - Software PWM via Timer2 ===== //
    DDRB |= (1 << SERVO_PIN);
    PORTB &= ~(1 << SERVO_PIN);

    // ===== Timer1 - Free-running for US sensor timing ===== //
    TCCR1A = 0;
    TCCR1B = (1 << CS11);  // Prescaler 8 (0.5us per tick)

    // ===== Timer2 - 1ms interrupt for millis() + servo PWM ===== //
    TCCR2A = (1 << WGM21);  // CTC mode
    TCCR2B = (1 << CS22) | (1 << CS20);  // Prescaler 128
    OCR2A = 124;  // 1ms
    TIMSK2 = (1 << OCIE2A);

    // ===== Ultrasonic sensors ===== //
    DDRB |= (1 << TRIG_PIN_LEFT) | (1 << TRIG_PIN_RIGHT);
    PORTB &= ~((1 << TRIG_PIN_LEFT) | (1 << TRIG_PIN_RIGHT));

    DDRD &= ~((1 << ECHO_PIN_LEFT) | (1 << ECHO_PIN_RIGHT));

    EICRA = (1 << ISC00) | (1 << ISC10);  // Any edge
    EIMSK = (1 << INT0) | (1 << INT1);

    sei();
}

int main() {
    setup();
    _delay_ms(100);
    
    i2c_init();
    ADC_init();
    /* mpu6050_init();
    calibrateIMU(); */
    
    uartPrint("System Initialized\r\n");
    
    unsigned long last_print = 0;
    unsigned long last_us = 0;
    
    while (1) {
        unsigned long now = millis();
        
        // Read sensors
        /* if (now - last_imu_time >= 10) {
            readIMU();
            yaw += gyrZ * 0.01;
            last_imu_time = now;
        } */
        
        if (now - last_us >= 60) {
            static uint8_t us_toggle = 0;
            if (us_toggle == 0) {
                triggerUS_Left();
                _delay_ms(30);
                left_distance = getDistance_Left();
                pulse_width_left = 0;
            } else {
                triggerUS_Right();
                _delay_ms(30);
                right_distance = getDistance_Right();
                pulse_width_right = 0;
            }
            us_toggle = !us_toggle;
            last_us = now;
        }
        
        ir_distance = readIR();
        
        // Print sensor data
        if (now - last_print >= 500) {
            uartPrint("L:");
            uartPrintInt(left_distance);
            uartPrint("cm R:");
            uartPrintInt(right_distance);
            uartPrint("cm IR:");
            uartPrintInt(ir_distance);
            uartPrint("cm | Yaw:");
            uartPrintFloat(yaw);
            uartPrint("\r\n");
            
            last_print = now;
        }

        startLiftFan();
        _delay_ms(20000);
    }
    
    return 0;
}