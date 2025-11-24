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

#define SERVO_PIN PB1       // OC1A

#define IR_PIN 0 // ADC0 (PC0)

#define IMU_ADDR 0x68

int system_state = 0; // 0 -> forward, 1 -> turning, 2 -> stopping
int turning_state = 0; // 1 -> right, 2 -> left
float system_yaw = 0;

// ---------------- IMU Variables ---------------- //
float gyro_scale = 65.5f;
float acc_scale = 8192.0f;

int16_t AccX, AccY, AccZ, GyrZ;
float accX, accY, accZ, gyrZ;
float yaw = 0.0f;
float offset_ax, offset_ay, offset_az, offset_gz;

unsigned long last_imu_time = 0;

// Ultrasonic sensor variables
volatile uint32_t pulse_width_left = 0;
volatile uint32_t pulse_width_right = 0;
volatile uint32_t echo_start_left = 0;
volatile uint32_t echo_start_right = 0;

uint32_t left_distance = 0;
uint32_t right_distance = 0;

uint16_t ir_distance = 0;

// Timing variables
volatile unsigned long milliseconds = 0;
volatile unsigned long timer0_overflow_count = 0;

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
    char buffer[12];
    ltoa(num, buffer, 10);
    uartPrint(buffer);
}

void uartPrintFloat(float num) {
    char buffer[20];
    dtostrf(num, 6, 2, buffer);
    uartPrint(buffer);
}

// ---------------- Timing Functions ---------------- //
// Timer0 overflow interrupt (for micros())
ISR(TIMER0_OVF_vect) {
    timer0_overflow_count++;
}

// Timer2 interrupt (for millis())
ISR(TIMER2_COMPA_vect) {
    milliseconds++;
}

unsigned long millis() {
    unsigned long m;
    cli();
    m = milliseconds;
    sei();
    return m;
}

unsigned long micros() {
    unsigned long m;
    uint8_t t;
    
    cli();
    m = timer0_overflow_count;
    t = TCNT0;
    
    // Check if overflow flag is set
    if ((TIFR0 & (1 << TOV0)) && (t < 255)) {
        m++;
    }
    sei();
    
    // Timer0 with prescaler 64: each tick = 4us
    // Overflow every 256 ticks = 1024us
    return (m * 1024UL) + (t * 4UL);
}

// ---------------- I2C Functions ---------------- //
void i2c_init(void) {
    TWSR = 0x00;
    TWBR = 72;  // 100kHz I2C for better stability
    TWCR = (1 << TWEN);  // Enable I2C
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

// ---------------- US Sensor Functions ---------------- //
ISR(INT0_vect) {
    if (PIND & (1 << ECHO_PIN_LEFT)) {
        // Rising edge - start timing
        echo_start_left = micros();
    } else {
        // Falling edge - calculate pulse width
        if (echo_start_left > 0) {
            uint32_t end_time = micros();
            if (end_time >= echo_start_left) {
                pulse_width_left = end_time - echo_start_left;
            } else {
                // Handle overflow (rare case)
                pulse_width_left = (0xFFFFFFFF - echo_start_left) + end_time;
            }
            echo_start_left = 0;
        }
    }
}

ISR(INT1_vect) {
    if (PIND & (1 << ECHO_PIN_RIGHT)) {
        // Rising edge - start timing
        echo_start_right = micros();
    } else {
        // Falling edge - calculate pulse width
        if (echo_start_right > 0) {
            uint32_t end_time = micros();
            if (end_time >= echo_start_right) {
                pulse_width_right = end_time - echo_start_right;
            } else {
                // Handle overflow (rare case)
                pulse_width_right = (0xFFFFFFFF - echo_start_right) + end_time;
            }
            echo_start_right = 0;
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
    if (pulse_width_left == 0 || pulse_width_left > 30000) {
        return 9999;
    }
    // Distance in cm = pulse_width_us / 58
    return pulse_width_left / 58;
}

uint32_t getDistance_Right() {
    if (pulse_width_right == 0 || pulse_width_right > 30000) {
        return 9999;
    }
    // Distance in cm = pulse_width_us / 58
    return pulse_width_right / 58;
}

// ---------------- Servo Control ---------------- //
void servo_init() {
    // Configure Timer1 for Fast PWM mode with ICR1 as TOP
    // This gives us precise control over the PWM frequency and pulse width
    
    DDRB |= (1 << SERVO_PIN);  // PB1 as output
    
    // Fast PWM mode, TOP = ICR1
    TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Clear OC1A on compare match, Fast PWM mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Fast PWM, prescaler = 8
    
    // Set TOP value for 20ms period (50Hz)
    // With 16MHz clock and prescaler 8: 16MHz / 8 = 2MHz = 0.5µs per tick
    // For 20ms: 20000µs / 0.5µs = 40000 ticks
    ICR1 = 40000;
    
    // Set initial position to center (1500µs)
    // 1500µs / 0.5µs = 3000 ticks
    OCR1A = 3000;
}

void set_servo_angle(float angle) {
    // Constrain angle to -90 to +90 degrees
    if (angle < -90.0f) angle = -90.0f;
    if (angle > 90.0f) angle = 90.0f;
    
    // Map angle to pulse width:
    // -90° → 1000µs → 2000 ticks
    //   0° → 1500µs → 3000 ticks
    // +90° → 2000µs → 4000 ticks
    
    // Formula: pulseWidth = 1500 + (angle * 500/90)
    // In timer ticks (0.5µs each): ticks = pulseWidth * 2
    
    float pulse_us = 1500.0f + (angle * (500.0f / 90.0f));
    uint16_t ticks = (uint16_t)(pulse_us * 2.0f);
    
    OCR1A = ticks;
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
    _delay_ms(100);  // Wait for MPU6050 to power up
    
    // Wake up MPU6050 (clear sleep bit)
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x6B);  // PWR_MGMT_1 register
    i2c_write(0x00);  // Clear sleep bit
    i2c_stop();
    _delay_ms(100);
    
    // Verify WHO_AM_I register
    uartPrint("Reading WHO_AM_I...\r\n");
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x75);  // WHO_AM_I register
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 1);
    uint8_t who_am_i = i2c_read_nack();
    i2c_stop();
    
    uartPrint("WHO_AM_I = 0x");
    char buf[4];
    itoa(who_am_i, buf, 16);
    uartPrint(buf);
    uartPrint(" (should be 0x68)\r\n");
    
    if (who_am_i != 0x68 && who_am_i != 0x72) {
        uartPrint("ERROR: MPU6050 not responding correctly!\r\n");
        return;
    }

    // Set gyroscope range to ±500°/s
    uartPrint("Configuring gyroscope...\r\n");
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1B);  // GYRO_CONFIG register
    i2c_write(0x08);  // ±500°/s (gyro_scale = 65.5)
    i2c_stop();
    _delay_ms(10);

    // Set accelerometer range to ±4g
    uartPrint("Configuring accelerometer...\r\n");
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1C);  // ACCEL_CONFIG register
    i2c_write(0x08);  // ±4g (acc_scale = 8192)
    i2c_stop();
    _delay_ms(10);
    
    uartPrint("MPU6050 initialized successfully!\r\n\r\n");
}

void readIMURaw() {
    uint8_t data[14];

    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x3B);  // Start reading from ACCEL_XOUT_H
    i2c_stop();
    
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 1);  // Read mode
    for (uint8_t i = 0; i < 13; i++) {
        data[i] = i2c_read_ack();
    }
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
    uartPrint("=== Calibrating IMU ===\r\n");
    uartPrint("Keep the sensor STILL!\r\n");
    
    float sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gz = 0;
    
    // First 100 readings are discarded
    for (int i = 0; i < 100; i++) {
        readIMURaw();
        _delay_ms(5);
    }
    
    // Next 500 readings for calibration
    uartPrint("Collecting calibration data...\r\n");
    for (int i = 0; i < 500; i++) {
        readIMURaw();
        sum_ax += accX;
        sum_ay += accY;
        sum_az += accZ;
        sum_gz += gyrZ;
        
        if (i % 100 == 0) {
            uartPrint(".");
        }
        _delay_ms(5);
    }
    
    offset_ax = sum_ax / 500.0f;
    offset_ay = sum_ay / 500.0f;
    offset_az = (sum_az / 500.0f) - 1.0f;  // Subtract 1g
    offset_gz = sum_gz / 500.0f;
    
    uartPrint("\r\nCalibration complete!\r\n");
    uartPrint("Offsets - AX:");
    uartPrintFloat(offset_ax);
    uartPrint(" AY:");
    uartPrintFloat(offset_ay);
    uartPrint(" AZ:");
    uartPrintFloat(offset_az);
    uartPrint(" GZ:");
    uartPrintFloat(offset_gz);
    uartPrint("\r\n\r\n");
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
    TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler 64 (for PWM and micros())
    OCR0A = 0;
    // Enable Timer0 overflow interrupt for micros()
    TIMSK0 = (1 << TOIE0);

    // ===== SERVO - Initialize with hardware PWM ===== //
    servo_init();

    // ===== Timer2 - 1ms interrupt for millis() ===== //
    TCCR2A = (1 << WGM21);  // CTC mode
    TCCR2B = (1 << CS22) | (1 << CS20);  // Prescaler 128
    OCR2A = 124;  // 1ms
    TIMSK2 = (1 << OCIE2A);

    // ===== Ultrasonic sensors ===== //
    DDRB |= (1 << TRIG_PIN_LEFT) | (1 << TRIG_PIN_RIGHT);
    PORTB &= ~((1 << TRIG_PIN_LEFT) | (1 << TRIG_PIN_RIGHT));

    DDRD &= ~((1 << ECHO_PIN_LEFT) | (1 << ECHO_PIN_RIGHT));

    EICRA = (1 << ISC00) | (1 << ISC10);  // Any edge on INT0 and INT1
    EIMSK = (1 << INT0) | (1 << INT1);    // Enable INT0 and INT1

    sei();
}

int main() {
    setup();
    _delay_ms(100);
    i2c_init();
    _delay_ms(50);
    ADC_init();
    mpu6050_init();
    _delay_ms(100);
    calibrateIMU();
    
    uartPrint("System Initialized!\r\n");
    
    unsigned long last_print = 0;
    unsigned long last_us = 0;

    startLiftFan();

    while (1) {
        // ---------------- Read Value ---------------- //
        unsigned long now = millis();
        
        // Read IMU sensors
        if (now - last_imu_time >= 10) {
            readIMU();
            yaw += gyrZ * 0.01;
            last_imu_time = now;
        }
        
        // Read ultrasonic sensors alternately
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
        
        // Read IR sensor
        ir_distance = readIR();
        
        // ---------------- Debugging ---------------- //
        if (now - last_print >= 500) {
            uartPrint("L:");
            uartPrintInt(left_distance);
            uartPrint("cm R:");
            uartPrintInt(right_distance);
            uartPrint("cm IR:");
            uartPrintInt(ir_distance);
            uartPrint("cm | Yaw:");
            uartPrintFloat(yaw);
            uartPrint("° | GyrZ:");
            uartPrintFloat(gyrZ);
            uartPrint(" AccX:");
            uartPrintFloat(accX);
            uartPrint("\r\n");
            
            last_print = now;
        }

        // ---------------- Algorithm ---------------- //
        switch (system_state)
        {
        case 0: // Forward state
            setThrustFan(200);

            // TODO : Stabilization using IMU

            // On hit wall -> turning decision then change state
            if (accX < 0.15 && accY < 0.15)
            {
                setThrustFan(0);
                if (left_distance > right_distance + 10.0)
                {
                    turning_state = 2;
                    system_state = 2;
                } else if (left_distance + 10.0 < right_distance) {
                    turning_state = 1;
                }
            }

            if (ir_distance < 50.0)
            {
                system_state = 2;
            }
            
            break;
        
        case 1: // Turning state
            if (turning_state == 2) // left
            {
                if (yaw < system_yaw - 90)
                {
                    set_servo_angle(-90);
                    setThrustFan(200);
                } else if(yaw >= system_state - 90){ // finished turning left
                    system_yaw -= 90;
                    set_servo_angle(0);
                    system_state = 0;
                }
            } else if(turning_state == 1) { // right
                if (yaw < system_yaw + 90)
                {
                    set_servo_angle(90);
                    setThrustFan(200);
                } else if(yaw >= system_state + 90){ // finished turning right
                    system_yaw += 90;
                    set_servo_angle(0);
                    system_state = 0;
                }
            } else {
                system_state = 0;
            }
            
            break;

        case 2: // Stopping state
            set_servo_angle(0);
            setThrustFan(0);
            stopLiftFan();
            break;
        
        default:
            break;
        }
    }
    
    return 0;
}